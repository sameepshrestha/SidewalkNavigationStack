"""
FrodoBots Perception — Clean API.

Usage from anywhere (e.g., ROS node, notebook, another script):

    import sys
    sys.path.insert(0, "/path/to/frodobot_perception_models")
    from api import PerceptionModel

    model = PerceptionModel.load("checkpoints/seg_head_iter_010000.pt")
    result = model.predict(rgb_numpy_image)
    # result.depth   → (H,W) float32 meters
    # result.mask    → (H,W) bool path mask
    # result.probs   → (C,H,W) float32 [0-1]
"""
import os
import sys
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from torchvision.transforms import functional as TF
from dataclasses import dataclass
from typing import Optional

# ── Internal path setup (so consumers don't have to) ────────────────────
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in [
    _PKG_DIR,
    os.path.join(_PKG_DIR, "model"),
    os.path.join(_PKG_DIR, "model", "src"),
    os.path.join(_PKG_DIR, "model", "src", "depth_anything_3"),
    os.path.join(_PKG_DIR, "dataset"),
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from multi_head_perception import MultiHeadPerception
from depth_anything_3.api import DepthAnything3
from navigable_dataset import ALL_CLASSES

# ── Constants ────────────────────────────────────────────────────────────
EMBED_DIMS = {
    "da3-small": 768,
    "da3-large": 2048,
    "da3metric-large": 1024,
}

CLASS_THRESH = {"path": 0.30}

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


@dataclass
class PerceptionResult:
    """Output of PerceptionModel.predict()."""
    depth: Optional[np.ndarray]    # (H, W) float32, metric depth in meters
    mask: np.ndarray               # (H, W) bool, path segmentation mask
    probs: np.ndarray              # (C, H, W) float32, per-class probabilities [0-1]
    logits: np.ndarray             # (C, H, W) float32, raw logits


class PerceptionModel:
    """
    Single entry point for depth + segmentation inference.

    Example:
        model = PerceptionModel.load("checkpoints/seg_head_iter_010000.pt")
        result = model.predict(cv2_rgb_image)
        navigable_mask = result.mask
        metric_depth = result.depth
    """

    def __init__(self, model: MultiHeadPerception, device: torch.device,
                 input_size=(294, 518)):
        self.model = model
        self.device = device
        self.input_size = input_size  # (H, W)
        self._mean = torch.tensor(IMAGENET_MEAN, device=device).view(1, 3, 1, 1)
        self._std = torch.tensor(IMAGENET_STD, device=device).view(1, 3, 1, 1)

    @classmethod
    def load(cls, checkpoint_path, model_name="da3metric-large",
             device="cuda", input_size=(294, 518)):
        """
        Load model from a checkpoint file.

        The checkpoint should contain backbone + seg_head weights
        (saved by train.py with the updated save_checkpoint).

        Args:
            checkpoint_path: path to .pt checkpoint file
            model_name: DA3 backbone variant
            device: 'cuda' or 'cpu'
            input_size: (H, W) model input resolution
        """
        device = torch.device(device if torch.cuda.is_available() else "cpu")

        # Resolve path relative to this package if not absolute
        if not os.path.isabs(checkpoint_path):
            checkpoint_path = os.path.join(_PKG_DIR, checkpoint_path)

        # Build model architecture
        da3 = DepthAnything3(model_name=model_name)
        model = MultiHeadPerception(
            da3_model=da3.model,
            num_seg_classes=len(ALL_CLASSES),
            dinov2_embed_dim=EMBED_DIMS[model_name],
        )

        # Load weights
        ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=True)

        if "seg_head_state_dict" in ckpt:
            model.seg_head.load_state_dict(ckpt["seg_head_state_dict"])
        else:
            model.seg_head.load_state_dict(ckpt)

        if "backbone_state_dict" in ckpt:
            model.backbone.load_state_dict(ckpt["backbone_state_dict"])
        else:
            print("⚠ No backbone weights in checkpoint — predictions may be wrong!")

        if "depth_head_state_dict" in ckpt:
            model.depth_head.load_state_dict(ckpt["depth_head_state_dict"])

        model = model.to(device).eval()
        return cls(model, device, input_size)

    def predict(self, image_rgb, run_depth=True, output_size=None) -> PerceptionResult:
        """
        Run depth + segmentation on a single image.

        Args:
            image_rgb: (H, W, 3) uint8 numpy array (RGB) or PIL Image
            run_depth: whether to compute metric depth
            output_size: (H, W) to resize outputs to. None = model input_size.

        Returns:
            PerceptionResult with depth, mask, probs, logits
        """
        if isinstance(image_rgb, np.ndarray):
            pil = Image.fromarray(image_rgb)
        else:
            pil = image_rgb

        H, W = self.input_size
        out_h = output_size[0] if output_size else H
        out_w = output_size[1] if output_size else W

        # Preprocess
        pil_resized = pil.resize((W, H), Image.BILINEAR)
        x = TF.to_tensor(pil_resized).unsqueeze(0).to(self.device)
        x = (x - self._mean) / self._std

        # Inference
        with torch.no_grad():
            out = self.model(x, run_depth=run_depth)

        # Segmentation
        seg_logits = out["segmentation"]
        if seg_logits.shape[2:] != (out_h, out_w):
            seg_logits = F.interpolate(seg_logits, size=(out_h, out_w),
                                       mode="bilinear", align_corners=False)
        logits_np = seg_logits.cpu().numpy()[0]
        probs_np = 1.0 / (1.0 + np.exp(-logits_np))
        mask = probs_np[0] >= CLASS_THRESH.get(ALL_CLASSES[0], 0.30)

        # Depth
        depth_np = None
        if run_depth and out.get("depth") is not None:
            d = out["depth"]
            if hasattr(d, "squeeze"):
                d = d.squeeze()
            if d.dim() == 3:
                d = d[0]
            if d.shape != (out_h, out_w):
                d = F.interpolate(d.unsqueeze(0).unsqueeze(0),
                                  size=(out_h, out_w), mode="bilinear",
                                  align_corners=False).squeeze()
            depth_np = d.cpu().numpy()

        return PerceptionResult(
            depth=depth_np,
            mask=mask,
            probs=probs_np,
            logits=logits_np,
        )

    def depth_to_pointcloud(self, depth, image_rgb, mask=None,
                            fx=500, fy=500, cx=None, cy=None):
        """
        Backproject depth to 3D point cloud.

        Returns: (points (N,3), colors (N,3), labels (N,) bool)
        """
        H, W = depth.shape
        cx = cx or W / 2.0
        cy = cy or H / 2.0

        u, v = np.meshgrid(np.arange(W), np.arange(H))
        Z = depth
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        valid = (Z > 0.01) & (Z < 100.0) & np.isfinite(Z)
        points = np.stack([X[valid], Y[valid], Z[valid]], axis=-1).astype(np.float32)

        if isinstance(image_rgb, Image.Image):
            image_rgb = np.array(image_rgb)
        if image_rgb.shape[:2] != (H, W):
            image_rgb = np.array(Image.fromarray(image_rgb).resize((W, H)))
        colors = image_rgb[valid]
        labels = mask[valid] if mask is not None else np.ones(len(points), dtype=bool)

        return points, colors, labels
