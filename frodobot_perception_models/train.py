"""
Train navigable surface segmentation using DA3 backbone + SimpleSegHead.
========================================================================
Trains the segmentation head on SAM3-generated pseudo-labels while keeping
the DINOv2 backbone and depth head frozen. Logs to Comet ML.

Usage:
    python train_seg.py                          # train with defaults
    python train_seg.py --batch_size 4           # custom batch size
    python train_seg.py --resume latest          # resume from latest checkpoint
    python train_seg.py --model_name da3-small   # use smaller backbone

Requirements:
    pip install comet_ml
    Set COMET_API_KEY env variable or paste in .comet.config
"""

import os
import sys
import time
import argparse
import matplotlib
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader
from comet_ml import start
from comet_ml.integration.pytorch import log_model

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt

SRC_DIR = os.path.dirname(os.path.abspath(__file__))

sys.path.insert(0, SRC_DIR)
sys.path.insert(0, os.path.join(SRC_DIR, "model"))
sys.path.insert(0, os.path.join(SRC_DIR, "model", "src"))
sys.path.insert(0, os.path.join(SRC_DIR, "model", "src", "depth_anything_3"))
sys.path.insert(0, os.path.join(SRC_DIR, "dataset"))
from multi_head_perception import MultiHeadPerception
from navigable_dataset import MultiClassSegDataset, ALL_CLASSES



def parse_args():
    p = argparse.ArgumentParser(description="Train navigable segmentation head")
    p.add_argument("--data_dirs", nargs="+", default=[
        "/home/sameep/phd_research/navigation_stack_compeition/Perception/dataset/image_mask_path_294x518_noqwen",
    ])
    p.add_argument("--train_size", type=int, nargs=2, default=[294, 518],
                   help="H W for training. Must be divisible by 14 (DINOv2 patch size)")
    p.add_argument("--num_workers", type=int, default=4)
    p.add_argument("--model_name", type=str, default="da3metric-large",
                   choices=["da3-small", "da3-large", "da3metric-large"],
                   help="DA3 backbone: da3-small (ViT-S), da3-large (ViT-L), or da3metric-large (ViT-L Metric)")
    p.add_argument("--da3_pretrained", type=str, default=None,
                   help="HF repo/local path for DA3 weights (used by DepthAnything3.from_pretrained)")
    p.add_argument("--num_classes", type=int, default=len(ALL_CLASSES),
                   help=f"Number of seg classes ({ALL_CLASSES})")
    p.add_argument("--imagenet_norm", action=argparse.BooleanOptionalAction, default=True,
                   help="Apply ImageNet normalization before DA3 backbone")
    p.add_argument("--batch_size", type=int, default=8)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--weight_decay", type=float, default=1e-4)
    p.add_argument("--max_iters", type=int, default=200_000)
    p.add_argument("--warmup_iters", type=int, default=500)
    p.add_argument("--use_lovasz", action=argparse.BooleanOptionalAction, default=True,
                   help="Add Lovasz hinge loss (IoU surrogate) on top of BCE+Dice")
    p.add_argument("--lovasz_weight", type=float, default=0.5,
                   help="Weight for Lovasz term when --use_lovasz is enabled")
    p.add_argument("--ckpt_dir", type=str,
                   default=os.path.join(SRC_DIR, "checkpoints"))
    p.add_argument("--save_every", type=int, default=10_000,
                   help="Save checkpoint every N iterations")
    p.add_argument("--val_every", type=int, default=1500,
                   help="Run validation every N iterations")
    p.add_argument("--resume", type=str, default=None,
                   help="'latest' or path to checkpoint .pt file")
    p.add_argument("--comet_project", type=str, default="navigable-segmentation")
    p.add_argument("--comet_workspace", type=str, default=None)
    p.add_argument("--log_every", type=int, default=50)

    return p.parse_args()


EMBED_DIMS = {
    "da3metric-large": 1024 # ViT-L Metric (1024, cat_token=False)
}


def build_model(model_name, num_classes, device, da3_pretrained=None):
    from depth_anything_3.api import DepthAnything3

    print(f"Loading DA3 backbone: {model_name}")
    if da3_pretrained:
        print(f"  Loading pretrained DA3 from: {da3_pretrained}")
        da3 = DepthAnything3.from_pretrained(da3_pretrained)
    else:
        print("  WARNING: no --da3_pretrained provided; DA3 backbone will be randomly initialized.")
        da3 = DepthAnything3(model_name=model_name)
    da3_net = da3.model  # DepthAnything3Net

    embed_dim = EMBED_DIMS[model_name]
    model = MultiHeadPerception(
        da3_model=da3_net,
        num_seg_classes=num_classes,
        dinov2_embed_dim=embed_dim,
    )
    model.freeze_depth_model()
    model = model.to(device)

    total = sum(p.numel() for p in model.parameters())
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"  Total params: {total:,}")
    print(f"  Trainable params: {trainable:,} (seg head only)")

    return model


def maybe_imagenet_normalize(images, enabled, mean_t, std_t):
    if not enabled:
        return images
    return (images - mean_t) / std_t




def lovasz_grad(gt_sorted):
    gts = gt_sorted.sum()
    intersection = gts - gt_sorted.float().cumsum(0)
    union = gts + (1 - gt_sorted).float().cumsum(0)
    jaccard = 1.0 - intersection / (union + 1e-6)
    if gt_sorted.numel() > 1:
        jaccard[1:] = jaccard[1:] - jaccard[:-1]
    return jaccard


def lovasz_hinge_flat(logits, labels):
    if labels.numel() == 0:
        return logits.sum() * 0.0
    signs = 2.0 * labels.float() - 1.0
    errors = 1.0 - logits * signs
    errors_sorted, perm = torch.sort(errors, dim=0, descending=True)
    gt_sorted = labels[perm]
    grad = lovasz_grad(gt_sorted)
    return torch.dot(F.relu(errors_sorted), grad)


def lovasz_hinge_multi_label(seg_logits, target_masks):
    losses = []
    for c in range(seg_logits.shape[1]):
        logits_c = seg_logits[:, c, :, :].reshape(-1)
        labels_c = target_masks[:, c, :, :].reshape(-1)
        losses.append(lovasz_hinge_flat(logits_c, labels_c))
    return torch.stack(losses).mean()


def compute_loss(seg_logits, target_masks, use_lovasz=False, lovasz_weight=0.5):
    target_masks_f = target_masks.float()
    bce = F.binary_cross_entropy_with_logits(
        seg_logits, target_masks_f, reduction="mean"
    )
    probs = torch.sigmoid(seg_logits)
    intersection = (probs * target_masks_f).sum(dim=(2,3))
    union = probs.sum(dim=(2,3)) + target_masks_f.sum(dim=(2,3))
    
    dice = 1 - (2 * intersection + 1e-6) / (union + 1e-6)
    dice = dice.mean()

    total = bce + dice
    if use_lovasz:
        lovasz = lovasz_hinge_multi_label(seg_logits, target_masks)
        total = total + lovasz_weight * lovasz
    return total



# ── Metrics ─────────────────────────────────────────────────────────────

@torch.no_grad()
def compute_metrics(seg_logits, target_masks, class_names):
    """Compute per-class IoU and mean IoU."""
    preds = (seg_logits.sigmoid() > 0.5).long()
    targets = target_masks.long()

    metrics = {}
    ious = []
    for i, name in enumerate(class_names):
        pred_c = preds[:, i]
        tgt_c = targets[:, i]
        intersection = (pred_c & tgt_c).sum().float()
        union = (pred_c | tgt_c).sum().float()
        iou = (intersection / (union + 1e-6)).item()
        ious.append(iou)
        metrics[f"iou/{name}"] = iou

    metrics["iou/mean"] = sum(ious) / len(ious)
    return metrics


# ── Checkpointing ───────────────────────────────────────────────────────

def save_checkpoint(model, optimizer, scheduler, iteration, loss, ckpt_dir, tag="latest"):
    """Save model checkpoint (seg head + backbone — backbone is frozen but needed for reproducibility)."""
    os.makedirs(ckpt_dir, exist_ok=True)
    path = os.path.join(ckpt_dir, f"seg_head_{tag}.pt")
    torch.save({
        "iteration": iteration,
        "seg_head_state_dict": model.seg_head.state_dict(),
        "backbone_state_dict": model.backbone.state_dict(),
        "depth_head_state_dict": model.depth_head.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "scheduler_state_dict": scheduler.state_dict() if scheduler else None,
        "loss": loss,
    }, path)
    print(f"  Checkpoint saved: {path}")
    return path


def load_checkpoint(model, optimizer, scheduler, ckpt_dir, resume):
    """Load checkpoint. Returns start iteration."""
    if resume == "latest":
        path = os.path.join(ckpt_dir, "seg_head_latest.pt")
    else:
        path = resume

    if not os.path.exists(path):
        print(f"  No checkpoint at {path}, starting fresh")
        return 0

    ckpt = torch.load(path, map_location="cpu", weights_only=True)
    model.seg_head.load_state_dict(ckpt["seg_head_state_dict"])
    # Restore backbone & depth head if saved (needed when backbone is randomly initialized)
    if "backbone_state_dict" in ckpt:
        model.backbone.load_state_dict(ckpt["backbone_state_dict"])
        print("  ✓ Restored backbone weights from checkpoint")
    if "depth_head_state_dict" in ckpt:
        model.depth_head.load_state_dict(ckpt["depth_head_state_dict"])
        print("  ✓ Restored depth head weights from checkpoint")
    optimizer.load_state_dict(ckpt["optimizer_state_dict"])
    if scheduler and ckpt.get("scheduler_state_dict"):
        scheduler.load_state_dict(ckpt["scheduler_state_dict"])
    start_iter = ckpt["iteration"]
    print(f"  Resumed from {path} at iteration {start_iter}")
    return start_iter


# ── Learning Rate Schedule ──────────────────────────────────────────────

def get_lr(iteration, max_iters, base_lr, warmup_iters):
    """Linear warmup + cosine decay."""
    if iteration < warmup_iters:
        return base_lr * (iteration + 1) / warmup_iters
    progress = (iteration - warmup_iters) / max(1, max_iters - warmup_iters)
    return base_lr * 0.5 * (1.0 + __import__("math").cos(__import__("math").pi * progress))


# ── Main Training Loop ─────────────────────────────────────────────────
import numpy as np

def plot_validation_results(images, targets, preds, class_names, out_path, experiment=None, step=0):
    """Plot validation image, ground-truth, and prediction."""
    # Convert first item in batch to numpy
    img = images[0].cpu().numpy().transpose(1, 2, 0) # [H, W, 3] RGB
    # Images are just [0, 1] tensors, so no un-normalization is needed
    img = np.clip(img, 0, 1)

    tgt = targets[0].cpu().numpy() # [C, H, W]
    prd = preds[0].cpu().numpy()   # [C, H, W]
    
    # Compress multi-class masks into a single vis map (argmax or sum)
    # Simple way: assign an integer to each class where it is 1
    tgt_vis = np.zeros(tgt.shape[1:])
    prd_vis = np.zeros(prd.shape[1:])
    for i in range(tgt.shape[0]):
        tgt_vis[tgt[i] == 1] = i + 1
        prd_vis[prd[i] > 0.0] = i + 1  # logit > 0 means prob > 0.5

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    axes[0].imshow(img)
    axes[0].set_title("Original RGB")
    axes[0].axis('off')
    
    axes[1].imshow(tgt_vis, cmap='tab10', vmin=0, vmax=len(class_names))
    axes[1].set_title("Ground Truth Mask")
    axes[1].axis('off')
    
    axes[2].imshow(prd_vis, cmap='tab10', vmin=0, vmax=len(class_names))
    axes[2].set_title("Predicted Mask")
    axes[2].axis('off')
    
    plt.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path)
    if experiment:
        experiment.log_image(out_path, name=f"val_plot_iter_{step}.png", step=step)
    plt.close(fig)

def main():
    args = parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # ── Comet ML ────────────────────────────────────────────────────────
    experiment = None
    try:
        experiment = start(
            api_key="fC6GESL10yERCjD7gVOyHc5S2",
            project_name="dp3-with-sam3",
            workspace="sameep54"
        )
        experiment.log_parameters(vars(args))
        print(f"  Comet experiment: {experiment.get_key()}")
    except Exception as e:
        print(f"  Comet ML not available ({e}), logging locally only")

    # ── Data ────────────────────────────────────────────────────────────
    train_size = tuple(args.train_size)
    print(f"\nLoading dataset (train_size={train_size})...")

    train_ds = MultiClassSegDataset(
        data_dirs=args.data_dirs,
        train_size=train_size,
        split="train",
        augment=True,
    )
    val_ds = MultiClassSegDataset(
        data_dirs=args.data_dirs,
        train_size=train_size,
        split="val",
        augment=False,
    )

    train_loader = DataLoader(
        train_ds, batch_size=args.batch_size, shuffle=True,
        num_workers=args.num_workers, pin_memory=True, drop_last=True,
    )
    val_loader = DataLoader(
        val_ds, batch_size=args.batch_size, shuffle=False,
        num_workers=args.num_workers, pin_memory=True,
    )

    print(f"  Train: {len(train_ds)}, Val: {len(val_ds)}")

    iters_per_epoch = len(train_loader)
    print(f"  Batches/epoch: {iters_per_epoch}")

    # ── Model ───────────────────────────────────────────────────────────
    print(f"\nBuilding model...")
    model = build_model(args.model_name, args.num_classes, device, args.da3_pretrained)
    mean_t = torch.tensor([0.485, 0.456, 0.406], device=device).view(1, 3, 1, 1)
    std_t = torch.tensor([0.229, 0.224, 0.225], device=device).view(1, 3, 1, 1)

    # ── Optimizer (only seg head params) ────────────────────────────────
    seg_params = [p for p in model.seg_head.parameters() if p.requires_grad]
    optimizer = torch.optim.AdamW(seg_params, lr=args.lr, weight_decay=args.weight_decay)
    scheduler = None  # using manual LR via get_lr()

    # ── Resume ──────────────────────────────────────────────────────────
    start_iter = 0
    if args.resume:
        start_iter = load_checkpoint(model, optimizer, scheduler, args.ckpt_dir, args.resume)

    # ── Training ────────────────────────────────────────────────────────
    print(f"\nTraining from iter {start_iter} to {args.max_iters}")
    print(f"  Checkpoint every {args.save_every} iters")
    print(f"  Validate every {args.val_every} iters")
    print(f"  Log every {args.log_every} iters")
    print("=" * 60)

    model.train()
    # Keep backbone in eval mode (frozen BatchNorm etc.)
    model.backbone.eval()

    train_iter = iter(train_loader)
    running_loss = 0.0
    t0 = time.time()

    for iteration in range(start_iter, args.max_iters):
        # Get batch (restart iterator at epoch boundary)
        try:
            images, masks = next(train_iter)
        except StopIteration:
            train_iter = iter(train_loader)
            images, masks = next(train_iter)

        images = images.to(device, non_blocking=True)
        masks = masks.to(device, non_blocking=True)

        # Update LR
        lr = get_lr(iteration, args.max_iters, args.lr, args.warmup_iters)
        for pg in optimizer.param_groups:
            pg["lr"] = lr

        # Forward (only use segmentation output, ignore depth)
        with torch.no_grad():
            # Backbone features are frozen, no grad needed
            model.backbone.eval()

        model_inputs = maybe_imagenet_normalize(images, args.imagenet_norm, mean_t, std_t)
        outputs = model(model_inputs, run_depth=False)
        seg_logits = outputs["segmentation"]  # [B, C, H, W]

        loss = compute_loss(
            seg_logits,
            masks,
            use_lovasz=args.use_lovasz,
            lovasz_weight=args.lovasz_weight,
        )

        # Backward
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(seg_params, max_norm=1.0)
        optimizer.step()

        running_loss += loss.item()

        # ── Logging ─────────────────────────────────────────────────────
        if (iteration + 1) % args.log_every == 0:
            avg_loss = running_loss / args.log_every
            elapsed = time.time() - t0
            its_per_sec = args.log_every / elapsed

            print(f"  iter {iteration+1:6d} | loss {avg_loss:.4f} | "
                  f"lr {lr:.2e} | {its_per_sec:.1f} it/s")

            if experiment:
                experiment.log_metric("train/loss", avg_loss, step=iteration+1)
                experiment.log_metric("train/lr", lr, step=iteration+1)
                experiment.log_metric("train/it_per_sec", its_per_sec, step=iteration+1)

            running_loss = 0.0
            t0 = time.time()

        # ── Validation ──────────────────────────────────────────────────
        if (iteration + 1) % args.val_every == 0:
            model.eval()
            val_loss = 0.0
            all_metrics = {}
            n_val = 0

            with torch.no_grad():
                for val_imgs, val_masks in val_loader:
                    val_imgs = val_imgs.to(device)
                    val_masks = val_masks.to(device)

                    val_inputs = maybe_imagenet_normalize(val_imgs, args.imagenet_norm, mean_t, std_t)
                    out = model(val_inputs, run_depth=False)
                    seg = out["segmentation"]
                    val_loss += compute_loss(
                        seg,
                        val_masks,
                        use_lovasz=args.use_lovasz,
                        lovasz_weight=args.lovasz_weight,
                    ).item()

                    batch_metrics = compute_metrics(seg, val_masks, ALL_CLASSES)
                    for k, v in batch_metrics.items():
                        all_metrics[k] = all_metrics.get(k, 0) + v
                    n_val += 1

                    # Limit val batches for speed
                    if n_val >= 50:
                        break

            val_loss /= n_val
            for k in all_metrics:
                all_metrics[k] /= n_val

            miou = all_metrics.get("iou/mean", 0)
            print(f"  ── VAL iter {iteration+1}: loss={val_loss:.4f}, mIoU={miou:.4f}")
            for cls in ALL_CLASSES:
                iou = all_metrics.get(f"iou/{cls}", 0)
                if iou > 0.001:
                    print(f"       {cls}: {iou:.4f}")

            if experiment:
                experiment.log_metric("val/loss", val_loss, step=iteration+1)
                for k, v in all_metrics.items():
                    experiment.log_metric(f"val/{k}", v, step=iteration+1)

            # Plotting
            plot_validation_results(
                val_imgs, val_masks, seg, 
                ALL_CLASSES, 
                f"{args.ckpt_dir}/val_plot_{iteration+1:06d}.png",
                experiment, step=iteration+1
            )
            model.train()
            model.backbone.eval()

        # ── Checkpoint ──────────────────────────────────────────────────
        if (iteration + 1) % args.save_every == 0:
            save_checkpoint(model, optimizer, scheduler, iteration+1,
                           loss.item(), args.ckpt_dir, tag=f"iter_{iteration+1:06d}")
            save_checkpoint(model, optimizer, scheduler, iteration+1,
                           loss.item(), args.ckpt_dir, tag="latest")

    # Final save
    save_checkpoint(model, optimizer, scheduler, args.max_iters,
                   loss.item(), args.ckpt_dir, tag="final")

    if experiment:
        experiment.end()

    print("\nTraining complete!")


if __name__ == "__main__":
    main()
