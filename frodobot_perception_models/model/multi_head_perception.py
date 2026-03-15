import os
import sys
import torch
import torch.nn as nn
import torch.nn.functional as F

# Ensure DA3 is in path so we can import from it easily
da3_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "depth_anything_3"))
if da3_path not in sys.path:
    sys.path.append(da3_path)

from depth_anything_3.model.da3 import DepthAnything3Net

class SimpleSegHead(nn.Module):
    """
    A lightweight, simple multi-scale decoding head for semantic segmentation.
    It takes multi-scale features from a backbone (like DINOv2), unifies their
    channel dimensions, concatenates them, and decodes into class logits.
    """
    def __init__(self, in_channels=(384, 384, 384, 384), num_classes=4, feature_size=256):
        super().__init__()
        
        # 1x1 convolutions to project all DINOv2 features to `feature_size`
        self.convs = nn.ModuleList([
            nn.Conv2d(c, feature_size, 1) for c in in_channels
        ])
        
        # Decoder blocks
        self.decoder = nn.Sequential(
            nn.Conv2d(len(in_channels) * feature_size, feature_size, 3, padding=1),
            nn.BatchNorm2d(feature_size),
            nn.ReLU(inplace=True),
            nn.Conv2d(feature_size, num_classes, 1)
        )

    def forward(self, features, H, W):
        upsampled_features = []
        # Target spatial size is the size of the earliest feature map (highest resolution)
        target_size = features[0].shape[2:]
        
        for i, feat in enumerate(features):
            feat_mapped = self.convs[i](feat)
            if feat_mapped.shape[2:] != target_size:
                # Upsample to target size
                feat_mapped = F.interpolate(feat_mapped, size=target_size, mode='bilinear', align_corners=False)
            upsampled_features.append(feat_mapped)
            
        # Concatenate features from all levels
        concat_features = torch.cat(upsampled_features, dim=1)
        
        # Decode and predict mask logits
        out = self.decoder(concat_features)
        
        # Upsample straight to original image resolution (H, W)
        out = F.interpolate(out, size=(H, W), mode='bilinear', align_corners=False)
        return out

class MultiHeadPerception(nn.Module):
    """
    A unified multi-head perception architecture that builds upon Depth Anything V3.
    It shares a single DINOv2 backbone to extract scene features, then branches into:
      1. A Depth Head (DA3's own DPTHead) for metric depth estimation.
      2. A Segmentation Head for predicting multi-class terrain/obstacle masks.
    """
    def __init__(self, da3_model: DepthAnything3Net, num_seg_classes=4, dinov2_embed_dim=384):
        '''
        Args:
          da3_model: An initialized DepthAnything3Net (e.g., DA3-Small).
          num_seg_classes: Classes for segmentation (e.g. 4 for curb, sidewalk, road, terrain).
          dinov2_embed_dim: The feature dimension output by the DINOv2 backbone. 
                            ViT-Small = 384, ViT-Base = 768, ViT-Large = 1024.
        '''
        super().__init__()
        
        # Share the heavy backbone (frozen or lightly tuned) and depth head
        self.backbone = da3_model.backbone
        self.depth_head = da3_model.head
        
        # DINOv2 typically outputs 4 intermediate feature maps for the decoders,
        # all of which have the same embedding dimension.
        in_channels = (dinov2_embed_dim,) * 4
        self.seg_head = SimpleSegHead(in_channels=in_channels, num_classes=num_seg_classes)
        
    def freeze_depth_model(self):
        """
        Freezes the DINOv2 backbone and the DA3 depth head so they do not
        update during training. Use this so you ONLY train the segmentation head.
        """
        for param in self.backbone.parameters():
            param.requires_grad = False
        for param in self.depth_head.parameters():
            param.requires_grad = False
            
        print("Frozen DINOv2 backbone and DA3 depth head. Only Seg Head will train.")
        
    def forward(self, x, run_depth=True):
        """
        x: Input image tensor of shape (B, 3, H, W)
        run_depth: Optional boolean to skip Depth Head calculation to save VRAM.
        Returns a dict: {"depth": <tensor (B,H,W)>, "segmentation": <tensor (B, classes, H, W)>}
        """
        if x.ndim == 4:
            B, C, H, W = x.shape
            x = x.unsqueeze(1)
            N = 1
        else:
            B, N, C, H, W = x.shape
            
        # 1. Extract features using DINOv2 backbone
        feats, aux_feats = self.backbone(x)
        
        # 2. Depth Head (Metric Depth) prediction
        if run_depth:
            with torch.autocast(device_type=x.device.type, enabled=False):
                depth_output = self.depth_head(feats, H, W, patch_start_idx=0)
        else:
            depth_output = None
            
        # 3. Process features
        spatial_feats = []
        for feat in feats:
            # DINOv2 returns a tuple (tensor,) or (t1, t2). The main feature is the first element
            if isinstance(feat, tuple) or isinstance(feat, list):
                feat = feat[0]
                
            if feat.ndim == 3: 
                h_feat = H // 14  # DA3 uses PATCH_SIZE = 14
                w_feat = W // 14
                num_tokens = h_feat * w_feat
                feat = feat[:, -num_tokens:]
                feat = feat.transpose(1, 2).reshape(B, -1, h_feat, w_feat)
            elif feat.ndim == 4 and feat.shape[1] == 1:
                feat = feat[:, 0]  # -> [B, tokens, C]
                h_feat = H // 14
                w_feat = W // 14
                num_tokens = h_feat * w_feat
                feat = feat[:, -num_tokens:]
                feat = feat.transpose(1, 2).reshape(B, -1, h_feat, w_feat)

            spatial_feats.append(feat)
            
        # 4. Segmentation Head prediction
        seg_logits = self.seg_head(spatial_feats, H, W)
        
        return {
            "depth": depth_output.depth if hasattr(depth_output, "depth") else depth_output,
            "segmentation": seg_logits
        }
