"""
PyTorch Dataset for Navigable Surface Segmentation.
=====================================================
Loads images and SAM3-generated masks for training a student segmentation
model (e.g., Depth Anything v3 fine-tuned for navigable surfaces).

Mask format (from segment_frodobots_multi / segment_output_rides):
    image_mask_2k/<ride>/masks/<frame>/road.png, sidewalk.png, ...
    image_mask_2k/<ride>/images/<frame>.jpg
    image_mask_2k/<ride>/scores/<frame>.json

Classes merged into binary "navigable" mask:
    navigable = road | sidewalk | path | crosswalk

Usage:
    from navigable_dataset import NavigableDataset
    ds = NavigableDataset(data_dirs=[...], train_size=(512, 512))
    img, mask = ds[0]
"""

import os
import glob
import json
import hashlib
import torch
import numpy as np
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms
from torchvision.transforms import functional as TF


# ── Configuration ───────────────────────────────────────────────────────

# Classes whose masks count as "navigable surface"
NAVIGABLE_CLASSES = ["path"]

# All classes produced by the segmentation pipeline
ALL_CLASSES = [ "path"]





class NavigableDataset(Dataset):
    """
    PyTorch Dataset that loads RGB images and binary navigable masks.

    Args:
        data_dirs:   List of mask output directories (image_mask, image_mask_2k, etc.)
        train_size:  (H, W) tuple to resize images and masks to. None = keep original.
        navigable_classes: Which mask classes to merge into the navigable label.
        require_navigable: If True, skip frames that have zero navigable pixels.
        split:       "train" or "val" — uses a deterministic 90/10 hash split.
        augment:     If True, apply training augmentations (flip, color jitter).
    """

    def __init__(
        self,
        data_dirs,
        train_size=None,
        navigable_classes=None,
        require_navigable=True,
        split="train",
        augment=False,
    ):
        self.train_size = train_size  # (H, W) or None
        self.navigable_classes = navigable_classes or NAVIGABLE_CLASSES
        self.require_navigable = require_navigable
        self.split = split
        self.augment = augment and (split == "train")

        # Discover all valid samples
        self.samples = self._discover_samples(data_dirs)
        print(f"NavigableDataset [{split}]: {len(self.samples)} samples "
              f"from {len(data_dirs)} dir(s), size={train_size}")

    def _discover_samples(self, data_dirs):
        """Scan directories and build list of (image_path, mask_dir, scores_path) tuples.
        
        Uses fast mask-file-size check instead of JSON parsing.
        Caches result to disk for instant subsequent loads.
        """
        import pickle

        # Build a cache key from directories
        cache_key = f"nav_ds_{self.split}_{'_'.join(sorted(self.navigable_classes))}"
        cache_key += f"_req{int(self.require_navigable)}"
        cache_dir = data_dirs[0] if data_dirs else "/tmp"
        cache_path = os.path.join(cache_dir, f".{cache_key}.cache")

        # Check if cache is still valid (all dirs exist with same mod time)
        if os.path.exists(cache_path):
            try:
                with open(cache_path, "rb") as f:
                    cached = pickle.load(f)
                if cached.get("dirs") == sorted(data_dirs):
                    print(f"  Loaded {len(cached['samples'])} samples from cache")
                    return cached["samples"]
            except Exception:
                pass

        samples = []
        skipped = 0

        for data_dir in data_dirs:
            if not os.path.isdir(data_dir):
                print(f"  ⚠ Skipping missing dir: {data_dir}")
                continue

            for ride_name in sorted(os.listdir(data_dir)):
                ride_path = os.path.join(data_dir, ride_name)
                images_dir = os.path.join(ride_path, "images")
                masks_dir = os.path.join(ride_path, "masks")
                scores_dir = os.path.join(ride_path, "scores")

                if not os.path.isdir(images_dir) or not os.path.isdir(masks_dir):
                    continue

                ride_count = 0
                for img_name in sorted(os.listdir(images_dir)):
                    if not img_name.endswith(".jpg"):
                        continue

                    frame_id = os.path.splitext(img_name)[0]
                    img_path = os.path.join(images_dir, img_name)
                    mask_dir = os.path.join(masks_dir, frame_id)
                    score_path = os.path.join(scores_dir, f"{frame_id}.json")

                    if not os.path.isdir(mask_dir):
                        continue

                    # Fast check: navigable mask PNG > ~100 bytes means non-empty
                    # (an all-zeros PNG is typically ~100-200 bytes, a real mask is larger)
                    if self.require_navigable:
                        has_nav = False
                        for cls_name in self.navigable_classes:
                            cls_path = os.path.join(mask_dir, f"{cls_name}.png")
                            try:
                                if os.path.getsize(cls_path) > 300:
                                    has_nav = True
                                    break
                            except OSError:
                                continue
                        if not has_nav:
                            skipped += 1
                            continue

                    # Deterministic train/val split (stable across Python runs)
                    h = int(hashlib.md5(frame_id.encode("utf-8")).hexdigest()[:8], 16) % 10
                    if self.split == "train" and h >= 9:
                        continue
                    if self.split == "val" and h < 9:
                        continue

                    samples.append((img_path, mask_dir, score_path))
                    ride_count += 1

                if ride_count > 0:
                    print(f"    {ride_name}: {ride_count} samples")

        print(f"  Total: {len(samples)} samples ({skipped} skipped, no navigable)")

        # Cache for next time
        try:
            with open(cache_path, "wb") as f:
                pickle.dump({"dirs": sorted(data_dirs), "samples": samples}, f)
        except Exception:
            pass

        return samples

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img_path, mask_dir, score_path = self.samples[idx]

        # Load image
        image = Image.open(img_path).convert("RGB")

        # Load and merge navigable masks into a single binary mask
        w, h = image.size
        nav_mask = np.zeros((h, w), dtype=np.uint8)

        for cls_name in self.navigable_classes:
            cls_mask_path = os.path.join(mask_dir, f"{cls_name}.png")
            if os.path.exists(cls_mask_path):
                cls_mask = np.array(Image.open(cls_mask_path).convert("L"))
                nav_mask = np.maximum(nav_mask, (cls_mask > 127).astype(np.uint8))

        mask = Image.fromarray(nav_mask * 255, mode="L")

        # Resize if needed
        if self.train_size is not None:
            th, tw = self.train_size
            image = image.resize((tw, th), Image.BILINEAR)
            mask = mask.resize((tw, th), Image.NEAREST)

        # Augmentation (training only)
        if self.augment:
            image, mask = self._augment(image, mask)

        # Convert to tensors
        image = TF.to_tensor(image)                            # [3, H, W] float [0, 1]
        mask = torch.from_numpy(np.array(mask) > 127).long()   # [H, W] binary {0, 1}

        return image, mask

    def _augment(self, image, mask):
        """Simple training augmentations that apply identically to image and mask."""
        # Random horizontal flip
        if torch.rand(1) > 0.5:
            image = TF.hflip(image)
            mask = TF.hflip(mask)

        # Color jitter (image only)
        image = transforms.ColorJitter(
            brightness=0.2, contrast=0.2, saturation=0.1, hue=0.05
        )(image)

        return image, mask

    def get_class_weights(self):
        """Estimate positive class weight from a sample of masks (for BCE loss)."""
        total_px = 0
        nav_px = 0
        n_samples = min(200, len(self.samples))

        for i in range(0, len(self.samples), max(1, len(self.samples) // n_samples)):
            _, mask_dir, _ = self.samples[i]
            for cls_name in self.navigable_classes:
                cls_path = os.path.join(mask_dir, f"{cls_name}.png")
                if os.path.exists(cls_path):
                    m = np.array(Image.open(cls_path).convert("L"))
                    nav_px += (m > 127).sum()
                    total_px += m.size
                    break
            else:
                # No navigable mask found, count as all-background
                img_path = self.samples[i][0]
                img = Image.open(img_path)
                total_px += img.width * img.height

        if nav_px == 0:
            return 1.0
        ratio = nav_px / total_px
        weight = (1 - ratio) / ratio
        return weight


# ── Convenience: multi-class dataset ────────────────────────────────────

class MultiClassSegDataset(NavigableDataset):
    """
    Returns per-class masks instead of a single binary navigable mask.

    __getitem__ returns (image, mask) where mask is [C, H, W] binary tensor
    with one channel per class in ALL_CLASSES.
    """

    def __init__(self, *args, classes=None, **kwargs):
        kwargs["require_navigable"] = False  # keep all frames
        super().__init__(*args, **kwargs)
        self.classes = classes or ALL_CLASSES

    def __getitem__(self, idx):
        img_path, mask_dir, _ = self.samples[idx]

        image = Image.open(img_path).convert("RGB")
        w, h = image.size

        masks = []
        for cls_name in self.classes:
            cls_path = os.path.join(mask_dir, f"{cls_name}.png")
            if os.path.exists(cls_path):
                m = np.array(Image.open(cls_path).convert("L"))
                masks.append((m > 127).astype(np.uint8))
            else:
                masks.append(np.zeros((h, w), dtype=np.uint8))

        # Resize
        if self.train_size is not None:
            th, tw = self.train_size
            image = image.resize((tw, th), Image.BILINEAR)
            resized = []
            for m in masks:
                rm = Image.fromarray(m * 255, mode="L").resize((tw, th), Image.NEAREST)
                resized.append(np.array(rm) > 127)
            masks = resized

        # Augment
        mask_pil = None
        if self.augment:
            # Stack masks and augment together
            if torch.rand(1) > 0.5:
                image = TF.hflip(image)
                masks = [np.fliplr(m) for m in masks]
            image = transforms.ColorJitter(
                brightness=0.2, contrast=0.2, saturation=0.1, hue=0.05
            )(image)

        image = TF.to_tensor(image)
        mask = torch.from_numpy(np.stack(masks, axis=0)).long()  # [C, H, W]

        return image, mask


# ── Quick test ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    DATA_DIRS = [
        "/home/sameep/phd_research/navigation_stack_compeition/Perception/dataset/image_mask",
        "/home/sameep/phd_research/navigation_stack_compeition/Perception/dataset/image_mask_2k",
    ]

    print("=" * 60)
    print("Testing NavigableDataset")
    print("=" * 60)

    ds_train = NavigableDataset(
        data_dirs=DATA_DIRS,
        train_size=(294, 518),
        split="train",
        augment=True,
    )
    ds_val = NavigableDataset(
        data_dirs=DATA_DIRS,
        train_size=(294, 518),
        split="val",
        augment=False,
    )

    print(f"\nTrain: {len(ds_train)}, Val: {len(ds_val)}")

    if len(ds_train) > 0:
        img, mask = ds_train[0]
        print(f"Image: {img.shape} {img.dtype}, range [{img.min():.2f}, {img.max():.2f}]")
        print(f"Mask:  {mask.shape} {mask.dtype}, unique={mask.unique().tolist()}")
        nav_pct = mask.float().mean() * 100
        print(f"Navigable: {nav_pct:.1f}% of pixels")

        weight = ds_train.get_class_weights()
        print(f"Suggested pos_weight for BCE: {weight:.2f}")

    print("\n" + "=" * 60)
    print("Testing MultiClassSegDataset")
    print("=" * 60)

    ds_multi = MultiClassSegDataset(
        data_dirs=DATA_DIRS,
        train_size=(294, 518),
        split="train",
    )
    print(f"Samples: {len(ds_multi)}")
    if len(ds_multi) > 0:
        img, mask = ds_multi[0]
        print(f"Image: {img.shape}, Mask: {mask.shape} (classes: {ALL_CLASSES})")
        for i, cls in enumerate(ALL_CLASSES):
            pct = mask[i].float().mean() * 100  
            if pct > 0:
                print(f"  {cls}: {pct:.1f}%")
