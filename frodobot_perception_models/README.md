# FrodoBots Perception

Depth estimation + path segmentation for the FrodoBots Earth Rover using [Depth Anything 3](https://github.com/DepthAnything/Depth-Anything-V3) (DA3) backbone with a trained segmentation head.

## Structure

```
frodobot_perception/
├── config/
│   └── perception.yaml       # shared config for training & inference
├── dataset/
│   └── navigable_dataset.py  # PyTorch Dataset for training
├── model/
│   ├── multi_head_perception.py   # MultiHeadPerception (backbone + depth + seg)
│   └── src/
│       └── depth_anything_3/      # DA3 model code
├── checkpoints/               # model weights (.pt files)
├── train.py                   # training script
├── inference.py               # video & image inference
├── pyproject.toml
└── README.md
```

## Quick Start

### Inference (single image or video)

```bash
# Video inference
python inference.py \
  --videos /path/to/video.mp4 \
  --checkpoint checkpoints/seg_head_iter_010000.pt \
  --model_name da3metric-large

# With pretrained DA3 backbone (if checkpoint doesn't contain backbone weights)
python inference.py \
  --videos /path/to/video.mp4 \
  --checkpoint checkpoints/seg_head_iter_010000.pt \
  --da3_pretrained depth-anything/DA3METRIC-LARGE
```

### Training

```bash
# Train with pretrained DA3 backbone (recommended)
python train.py \
  --da3_pretrained depth-anything/DA3METRIC-LARGE \
  --data_dirs /path/to/dataset \
  --max_iters 200000

# Resume from checkpoint
python train.py \
  --da3_pretrained depth-anything/DA3METRIC-LARGE \
  --resume latest
```

### Python API

```python
from frodobot_perception.inference import PerceptionModel

# Load model
model = PerceptionModel.from_checkpoint("checkpoints/seg_head_iter_010000.pt")

# Run inference
import numpy as np
from PIL import Image
img = np.array(Image.open("test.jpg").convert("RGB"))
result = model.predict(img)

# Access results
depth = result['depth']        # (H, W) float32, meters
seg_mask = result['seg_mask']  # (H, W) bool
seg_probs = result['seg_probs'] # (C, H, W) float32 [0-1]

# Generate point cloud
points, colors, labels = model.depth_to_pointcloud(depth, img, seg_mask)
```

## Model Architecture

- **Backbone**: DINOv2 ViT-L (from DA3, frozen)
- **Depth Head**: DPT metric depth (from DA3, frozen)
- **Seg Head**: SimpleSegHead (trained, ~3.4M params)

Only the segmentation head is trained. The backbone and depth head use pretrained weights from `depth-anything/DA3METRIC-LARGE`.

## Classes

Currently trained for single-class binary segmentation:
- `path` — navigable walking/driving path

## Requirements

```bash
pip install -e .                     # core inference
pip install -e ".[train]"            # + training deps (comet_ml, matplotlib)
pip install -e ".[viz]"              # + visualization (open3d)
```
