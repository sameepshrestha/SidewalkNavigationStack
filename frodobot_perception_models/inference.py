#!/usr/bin/env python3
import os
import sys
import json
import glob
import argparse
import cv2
import torch
import numpy as np
from PIL import Image, ImageDraw
from torchvision.transforms import functional as TF

SRC_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(SRC_DIR, "model")
DA3_DIR = os.path.join(MODEL_DIR, "src", "depth_anything_3")

sys.path.insert(0, SRC_DIR)
sys.path.insert(0, MODEL_DIR)
sys.path.insert(0, os.path.join(MODEL_DIR, "src"))
sys.path.insert(0, DA3_DIR)
sys.path.insert(0, os.path.join(SRC_DIR, "dataset"))

from multi_head_perception import MultiHeadPerception
from navigable_dataset import ALL_CLASSES

COLORS = [
    (0, 0, 255),
    (0, 255, 0),
    (255, 255, 0),
    (255, 0, 255),
    (255, 0, 0),
    (0, 255, 255),
]

EMBED_DIMS = {
    "da3-small": 768,
    "da3-large": 2048,
    "da3metric-large": 1024,
}

CLASS_THRESH = {
    "road": 0.30,
    "sidewalk": 0.30,
    "grass": 0.35,
    "crosswalk": 0.30,
    "person": 0.40,
    "path": 0.30,
}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--videos", nargs="+", required=True)
    p.add_argument("--checkpoint", type=str, required=True)
    p.add_argument("--model_name", type=str, default="da3metric-large")
    p.add_argument("--da3_pretrained", type=str, default=None,
                   help="HF repo/local path for DA3 weights (used by DepthAnything3.from_pretrained)")
    p.add_argument("--output_dir", type=str, default=os.path.join(SRC_DIR, "inference_output_fixed"))
    p.add_argument("--input_size", type=int, nargs=2, default=[294, 518])  # H W
    p.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--max_frames", type=int, default=-1)
    p.add_argument("--start_sec", type=float, default=0.0, help="Start processing video from this second")
    p.add_argument("--depth_vis_max", type=float, default=20.0, help="Max depth in meters for visualizer mapping")
    p.add_argument("--min_area_percent", type=float, default=0.3)
    p.add_argument("--focal_length_px", type=float, default=250.50447, help="Focal length for real metric depth (fx+fy)/2")
    p.add_argument("--proxy_to_metric_divisor", type=float, default=300.0,
                   help="Depth scaling divisor for non-metric DA3 variants")
    return p.parse_args()


def load_model(model_name, checkpoint_path, device, da3_pretrained=None):
    from depth_anything_3.api import DepthAnything3

    if da3_pretrained:
        print(f"Loading pretrained DA3 from: {da3_pretrained}")
        da3 = DepthAnything3.from_pretrained(da3_pretrained)
    else:
        print("WARNING: no --da3_pretrained provided; DA3 backbone will be randomly initialized.")
        da3 = DepthAnything3(model_name=model_name)
    model = MultiHeadPerception(
        da3_model=da3.model,
        num_seg_classes=len(ALL_CLASSES),
        dinov2_embed_dim=EMBED_DIMS[model_name],
    )

    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    if "seg_head_state_dict" in ckpt:
        model.seg_head.load_state_dict(ckpt["seg_head_state_dict"])
    else:
        model.seg_head.load_state_dict(ckpt)

    # Restore backbone & depth head if saved (critical when backbone is randomly initialized)
    if "backbone_state_dict" in ckpt:
        model.backbone.load_state_dict(ckpt["backbone_state_dict"])
        print("  ✓ Restored backbone weights from checkpoint")
    else:
        print("  ⚠ WARNING: No backbone weights in checkpoint! Backbone is randomly initialized.")
        print("    Segmentation predictions will be WRONG unless pretrained weights are used.")
    if "depth_head_state_dict" in ckpt:
        model.depth_head.load_state_dict(ckpt["depth_head_state_dict"])
        print("  ✓ Restored depth head weights from checkpoint")

    model = model.to(device)
    model.eval()
    return model


def depth_to_vis(depth, depth_vis_max):
    if depth.ndim == 3:
        if depth.shape[0] == 1:
            depth = depth[0]
        elif depth.shape[-1] == 1:
            depth = depth[..., 0]

    depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    depth = np.clip(depth, 0.0, depth_vis_max)
    depth_u8 = (255.0 * depth / max(depth_vis_max, 1e-6)).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)
    return cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)


def make_overlay(rgb, probs, min_area_percent=0.3):
    h, w = rgb.shape[:2]
    overlay = Image.fromarray(rgb).convert("RGBA")
    draw = ImageDraw.Draw(overlay)

    stats = {}
    y = 5

    for i, cls in enumerate(ALL_CLASSES):
        prob = probs[i]
        thr = CLASS_THRESH.get(cls, 0.30)
        mask = prob >= thr

        area_percent = 100.0 * float(mask.mean())
        mean_conf = float(prob[mask].mean()) if mask.any() else 0.0
        max_conf = float(prob.max())

        stats[cls] = {
            "threshold": float(thr),
            "area_percent": area_percent,
            "mean_conf_on_mask": mean_conf,
            "max_conf": max_conf,
            "pixels": int(mask.sum()),
        }

        if area_percent > min_area_percent:
            color_layer = np.zeros((h, w, 4), dtype=np.uint8)
            color_layer[mask] = COLORS[i] + (100,)
            overlay = Image.alpha_composite(overlay, Image.fromarray(color_layer, mode="RGBA"))
            draw = ImageDraw.Draw(overlay)
            draw.text((5, y), f"{cls}: {area_percent:.1f}% | conf {mean_conf:.2f}", fill=(255, 255, 255, 255))
            y += 16

    return np.array(overlay.convert("RGB")), stats


def process_video(video_path, model, args):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Could not open {video_path}")
        return

    if args.start_sec > 0:
        fps = cap.get(cv2.CAP_PROP_FPS)
        start_frame = int(args.start_sec * fps)
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        print(f"Starting at second {args.start_sec} (frame {start_frame})")
    else:
        start_frame = 0

    vid_name = os.path.splitext(os.path.basename(video_path))[0]
    out_dir = os.path.join(args.output_dir, vid_name)
    os.makedirs(out_dir, exist_ok=True)

    images_dir = os.path.join(out_dir, "images")
    depths_dir = os.path.join(out_dir, "depths")
    masks_dir = os.path.join(out_dir, "masks")
    probs_dir = os.path.join(out_dir, "prob_maps")
    vis_dir = os.path.join(out_dir, "visualizations")
    scores_dir = os.path.join(out_dir, "scores")

    for d in [images_dir, depths_dir, masks_dir, probs_dir, vis_dir, scores_dir]:
        os.makedirs(d, exist_ok=True)

    frame_idx = 0
    processed = 0

    while True:
        ret, frame_bgr = cap.read()
        if not ret:
            break
        if args.max_frames > 0 and processed >= args.max_frames:
            break
        if frame_idx % args.stride != 0:
            frame_idx += 1
            continue

        actual_frame = start_frame + frame_idx
        frame_id = f"{vid_name}_f{actual_frame:05d}"
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(rgb)
        
        orig_w, orig_h = pil.size
        # Resize to network input shape (H, W)
        net_img = pil.resize((args.input_size[1], args.input_size[0]), Image.BILINEAR)
        x = TF.to_tensor(net_img)
        x = TF.normalize(x, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        x = x.unsqueeze(0).to(args.device)

        with torch.no_grad():
            out = model(x, run_depth=True)
            depth = out["depth"].detach().cpu().numpy()[0]
            seg_logits = out["segmentation"]
            
            print(f"Max depth logit = {depth.max():.2f}")
            print(f"Max seg logit = {seg_logits.max().item():.2f}")
            probs = torch.sigmoid(seg_logits).detach().cpu().numpy()[0]

        # Upscale back to original video dimensions
        if depth.ndim == 3 and depth.shape[0] == 1:
            depth = depth[0]
        elif depth.ndim == 3 and depth.shape[-1] == 1:
            depth = depth[..., 0]
        depth = cv2.resize(depth, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        
        # Metric models already predict metric depth. Only scale proxy depth variants.
        if "metric" not in args.model_name:
            depth = (args.focal_length_px * depth) / args.proxy_to_metric_divisor

        out_probs = []
        for c in range(probs.shape[0]):
            out_probs.append(cv2.resize(probs[c], (orig_w, orig_h), interpolation=cv2.INTER_LINEAR))
        probs = np.stack(out_probs, axis=0)

        pil.save(os.path.join(images_dir, f"{frame_id}.jpg"))

        np.savez_compressed(os.path.join(depths_dir, f"{frame_id}.npz"), depth=depth.astype(np.float32))
        Image.fromarray(depth_to_vis(depth, args.depth_vis_max)).save(
            os.path.join(depths_dir, f"{frame_id}_vis.png")
        )

        frame_mask_dir = os.path.join(masks_dir, frame_id)
        frame_prob_dir = os.path.join(probs_dir, frame_id)
        os.makedirs(frame_mask_dir, exist_ok=True)
        os.makedirs(frame_prob_dir, exist_ok=True)

        for i, cls in enumerate(ALL_CLASSES):
            thr = CLASS_THRESH.get(cls, 0.30)
            mask = (probs[i] >= thr).astype(np.uint8)
            Image.fromarray(mask * 255).save(os.path.join(frame_mask_dir, f"{cls}.png"))
            np.savez_compressed(os.path.join(frame_prob_dir, f"{cls}_prob.npz"), prob=probs[i].astype(np.float32))

        overlay, stats = make_overlay(rgb, probs, min_area_percent=args.min_area_percent)
        depth_vis = depth_to_vis(depth, args.depth_vis_max)
        combo = np.hstack([rgb, depth_vis, overlay])
        Image.fromarray(combo).save(os.path.join(vis_dir, f"{frame_id}_vis.jpg"))

        with open(os.path.join(scores_dir, f"{frame_id}.json"), "w") as f:
            json.dump(stats, f, indent=2)

        if processed % 10 == 0:
            print(f"frame {frame_idx} | processed {processed + 1}")

        frame_idx += 1
        processed += 1

    cap.release()


def collect_videos(items):
    vids = []
    for item in items:
        if os.path.isdir(item):
            vids.extend(sorted(glob.glob(os.path.join(item, "*.mp4"))))
        else:
            vids.append(item)
    return vids


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    model = load_model(args.model_name, args.checkpoint, args.device, args.da3_pretrained)
    for vp in collect_videos(args.videos):
        process_video(vp, model, args)


if __name__ == "__main__":
    main()
