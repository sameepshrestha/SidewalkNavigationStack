#!/usr/bin/env python3
import sys
import os

# --- PATH FIX ---
DA3_PATH = "/home/sameep/phd_research/sidewalkauto_ws/src/Depth-Anything-3/src"
if os.path.exists(DA3_PATH): sys.path.append(DA3_PATH)
else: sys.path.append("/home/sameep/phd_research/sidewalkauto_ws/src/Depth-Anything-3")
# ----------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

from depth_anything_3.api import DepthAnything3
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

class DepthAnything3Node(Node):
    def __init__(self):
        super().__init__('da3_node')
        self.get_logger().info("Loading AI Models...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # 1. Models
        self.model = DepthAnything3.from_pretrained("depth-anything/DA3Metric-Large").to(self.device)
        self.seg_processor = SegformerImageProcessor.from_pretrained("nvidia/segformer-b1-finetuned-cityscapes-1024-1024")
        self.seg_model = SegformerForSemanticSegmentation.from_pretrained("nvidia/segformer-b1-finetuned-cityscapes-1024-1024").to(self.device)
        self.seg_model.eval()

        self.cv_bridge = CvBridge()
        
        # 2. Topics
        self.subscription = self.create_subscription(Image, '/camera/front/image_raw', self.listener_callback, 10)
        self.pub_depth = self.create_publisher(Image, '/da3/depth', 10)
        self.pub_seg = self.create_publisher(Image, '/semantic_cam/image_rect_color', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/front/camera_info', 10)
        
        # 3. Your Calibration
        self.calib_K = [256.7347218458715, 0.0, 325.1050532995314, 0.0, 255.04767516575092, 242.39722332171107, 0.0, 0.0, 1.0]
        self.calib_P = [256.7347218458715, 0.0, 325.1050532995314, 0.0, 0.0, 255.04767516575092, 242.39722332171107, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.calib_D = [-0.019323648968244223, -0.016212256399139098, -0.005261646209074583, 0.0025308961188848897, 0.0]

    def listener_callback(self, msg):
        # Input Image (Original Resolution)
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        orig_h, orig_w = cv_img.shape[:2]

        # A. Inference
        pred = self.model.inference([cv_img]) 
        depth = pred.depth[0]

        # FIX 1: RESIZE DEPTH TO MATCH CAMERA
        # The model often changes resolution (e.g. 504x378 vs 640x480). 
        # We must resize depth back to the original camera size so they match.
        if depth.shape[0] != orig_h or depth.shape[1] != orig_w:
            depth = cv2.resize(depth, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)

        # Segformer Inference
        inputs = self.seg_processor(images=cv_img, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.seg_model(**inputs)
        mask = torch.nn.functional.interpolate(outputs.logits, size=(orig_h, orig_w), mode='bilinear').argmax(dim=1)[0].cpu().numpy()

        # B. Color Map
        seg_color = np.full((orig_h, orig_w, 3), [255, 255, 255], dtype=np.uint8) 

        # Now apply your specific labels on top
        seg_color[mask == 0] = [0, 0, 255]   # Road = Red
        seg_color[mask == 1] = [0, 255, 0] 
        
        # Ensure Header & Frame ID match everything
        header = msg.header
        if not header.frame_id: header.frame_id = "camera_front_optical_frame"

        # C. Publish Images
        d_msg = self.cv_bridge.cv2_to_imgmsg(depth, "32FC1")
        d_msg.header = header
        self.pub_depth.publish(d_msg)

        s_msg = self.cv_bridge.cv2_to_imgmsg(seg_color, "bgr8")
        s_msg.header = header
        self.pub_seg.publish(s_msg)

        # D. Publish Camera Info
        i_msg = CameraInfo()
        i_msg.header = header  # <--- FIX 2: THIS HEADER WAS GETTING DELETED IN FALLBACK
        i_msg.height = orig_h
        i_msg.width = orig_w
        i_msg.distortion_model = "plumb_bob"

        # Default to Fallback
        use_fallback = True

        # Check AI Intrinsics
        if hasattr(pred, 'intrinsics') and pred.intrinsics is not None:
             k_ai = pred.intrinsics[0]
             if np.sum(k_ai) > 1:
                 use_fallback = False
                 i_msg.d = [0.0]*5
                 i_msg.k = k_ai.flatten().tolist()
                 # Identity Rotation
                 i_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] 
                 # Projection [K | 0]
                 i_msg.p = [k_ai[0,0], 0., k_ai[0,2], 0., 0., k_ai[1,1], k_ai[1,2], 0., 0., 0., 1., 0.]

        # Apply Fallback if needed
        if use_fallback:
             # Notice: We simply fill the fields on 'i_msg', we DO NOT create a new object.
             i_msg.d = self.calib_D
             i_msg.k = self.calib_K
             i_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] # Identity
             i_msg.p = self.calib_P

        self.pub_info.publish(i_msg)

def main():
    rclpy.init()
    rclpy.spin(DepthAnything3Node())
    rclpy.shutdown()