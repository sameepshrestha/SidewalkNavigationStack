#!/usr/bin/env python3
"""
FrodoBots Perception ROS 2 Node.

Subscribes to camera images, runs depth + segmentation inference,
publishes depth, segmentation mask, overlay, and traversable-path point cloud.

The point cloud can be published in:
  - camera_optical_frame (let TF2 handle transform — live mode)
  - base_link (hardcoded transform — standalone/video mode)

Modes:
  - Live: subscribes to /earth_rover/front/image_raw (from frodobot_bridge)
  - Video: run with video_file param to replay an MP4

Usage:
  ros2 run frodobot_perception_ros perception_node --ros-args \
    --params-file config/perception.yaml \
    -p video_file:=/path/to/video.mp4
     source /home/sameep/phd_research/navigation_stack_compeition/stackenv/bin/activate \
     && source install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
"""
import os
import sys
import math

# --- Numpy / ROS 2 Path Conflict Fix ---
# ros2 run puts /opt/ros/humble before the active virtualenv in sys.path.
# This causes Python to load numpy 2.x from system instead of 1.x from the venv.
# We force the virtualenv's site-packages to the absolute front.
venv_path = os.path.join(
    os.getenv("HOME", ""),
    "phd_research/navigation_stack_compeition/stackenv/lib/python3.10/site-packages"
)
if os.path.isdir(venv_path) and venv_path in sys.path:
    sys.path.remove(venv_path)
    sys.path.insert(0, venv_path)

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

MODELS_DIR = os.path.join(
    os.getenv("HOME"), "phd_research", "navigation_stack_compeition",
    "frodobot_perception_models"
)
MODELS_DIR = os.path.normpath(MODELS_DIR)
if MODELS_DIR not in sys.path:
    sys.path.insert(0, MODELS_DIR)

from api import PerceptionModel  # noqa: E402


def make_rotation_matrix(pitch_rad):

    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    R = np.array([
        [sp,    0,  cp],
        [-1,    0,   0],
        [0,   -cp,   sp],
    ], dtype=np.float32)

    R_opt = np.array([
        [0,  0,  1],
        [-1, 0,  0],
        [0, -1,  0],
    ], dtype=np.float32)

    R_pitch = np.array([
        [cp,  0, sp],
        [0,   1,  0],
        [-sp, 0, cp],
    ], dtype=np.float32)

    return R_pitch @ R_opt


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.declare_parameter("checkpoint", os.path.join(
            MODELS_DIR, "checkpoints", "seg_head_iter_010000.pt"))
        self.declare_parameter("model_name", "da3metric-large")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("input_size_h", 294)
        self.declare_parameter("input_size_w", 518)
        self.declare_parameter("seg_threshold", 0.30)
        self.declare_parameter("image_topic", "/earth_rover/front/image_raw")
        self.declare_parameter("video_file", "")
        self.declare_parameter("video_fps", 10.0)
        self.declare_parameter("video_stride", 1)
        self.declare_parameter("camera_fx", 500.0)
        self.declare_parameter("camera_fy", 500.0)
        self.declare_parameter("camera_cx", 512.0)
        self.declare_parameter("camera_cy", 288.0)
        self.declare_parameter("camera_frame", "front_camera_optical_frame")
        self.declare_parameter("transform_to_base", True)
        self.declare_parameter("camera_x", 0.16)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.14)
        self.declare_parameter("camera_pitch_deg", 8.0)
        self.declare_parameter("publish_pointcloud", True)
        self.declare_parameter("pc_min_depth", 0.1)
        self.declare_parameter("pc_max_depth", 20.0)
        self.declare_parameter("pc_downsample", 1)
        checkpoint = self.get_parameter("checkpoint").value
        if not checkpoint:
            checkpoint = os.path.join(MODELS_DIR, "checkpoints", "seg_head_iter_010000.pt")
        model_name = self.get_parameter("model_name").value
        device = self.get_parameter("device").value
        input_h = self.get_parameter("input_size_h").value
        input_w = self.get_parameter("input_size_w").value
        self.seg_threshold = self.get_parameter("seg_threshold").value
        self.image_topic = self.get_parameter("image_topic").value
        self.video_file = self.get_parameter("video_file").value
        self.video_fps = self.get_parameter("video_fps").value
        self.video_stride = self.get_parameter("video_stride").value

        self.fx = self.get_parameter("camera_fx").value
        self.fy = self.get_parameter("camera_fy").value
        self.cx = self.get_parameter("camera_cx").value
        self.cy = self.get_parameter("camera_cy").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.transform_to_base = self.get_parameter("transform_to_base").value
        cam_x = self.get_parameter("camera_x").value
        cam_y = self.get_parameter("camera_y").value
        cam_z = self.get_parameter("camera_z").value
        pitch_deg = self.get_parameter("camera_pitch_deg").value

        self.publish_pc = self.get_parameter("publish_pointcloud").value
        self.pc_min_depth = self.get_parameter("pc_min_depth").value
        self.pc_max_depth = self.get_parameter("pc_max_depth").value
        self.pc_downsample = max(1, int(self.get_parameter("pc_downsample").value))
        if self.transform_to_base:
            pitch_rad = math.radians(pitch_deg)
            self.R_cam_to_base = make_rotation_matrix(pitch_rad)
            self.t_cam_to_base = np.array([cam_x, cam_y, cam_z], dtype=np.float32)
            self.pc_frame = "base_link"
            self.get_logger().info(
                f"Transform to base_link: translation=[{cam_x}, {cam_y}, {cam_z}], "
                f"pitch={pitch_deg}°")
        else:
            self.R_cam_to_base = None
            self.t_cam_to_base = None
            self.pc_frame = self.camera_frame
        #lloading the modedel here
        self.get_logger().info(f"Loading model: {model_name}")
        self.get_logger().info(f"Checkpoint: {checkpoint}")
        self.model = PerceptionModel.load(
            checkpoint, model_name=model_name,
            device=device, input_size=(input_h, input_w)
        )
        self.get_logger().info("Model loaded ✓")
        self.get_logger().info(
            f"Camera intrinsics: fx={self.fx}, fy={self.fy}, "
            f"cx={self.cx}, cy={self.cy}")

        self.depth_pub = self.create_publisher(Image, "/perception/depth", 5)
        self.seg_pub = self.create_publisher(Image, "/perception/segmentation", 5)
        self.overlay_pub = self.create_publisher(Image, "/perception/overlay", 5)
        if self.publish_pc:
            self.pc_pub = self.create_publisher(
                PointCloud2, "/perception/traversable_cloud", 5)
        if self.video_file:
            self.get_logger().info(f"VIDEO MODE: replaying {self.video_file}")
            self.cap = cv2.VideoCapture(self.video_file)
            if not self.cap.isOpened():
                self.get_logger().error(f"Cannot open video: {self.video_file}")
                return
            self.video_frame_idx = 0
            self.image_pub = self.create_publisher(Image, self.image_topic, 5)
            self.timer = self.create_timer(1.0 / self.video_fps, self.video_callback)
        else:
            self.get_logger().info(f"LIVE MODE: subscribing to {self.image_topic}")
            self.cap = None
            self.sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data)

    def video_callback(self):
        for _ in range(self.video_stride):
            ret, frame_bgr = self.cap.read()
            self.video_frame_idx += 1
            if not ret:
                self.get_logger().info("Video ended.")
                self.timer.cancel()
                return

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img_msg = self._numpy_to_image_msg(rgb, "rgb8", self.camera_frame)
        self.image_pub.publish(img_msg)
        self._run_inference(rgb, img_msg.header)

        if self.video_frame_idx % 50 == 0:
            self.get_logger().info(f"Video frame {self.video_frame_idx}")

    def image_callback(self, msg: Image):
        if msg.encoding in ("bgr8", "BGR8"):
            bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        elif msg.encoding in ("rgb8", "RGB8"):
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        else:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return
        self._run_inference(rgb, msg.header)

    def _run_inference(self, rgb: np.ndarray, header: Header):
        orig_h, orig_w = rgb.shape[:2]
        result = self.model.predict(rgb, run_depth=True,
                                     output_size=(orig_h, orig_w))
    
        # 1. Depth image (32FC1)
        if result.depth is not None:
            depth_msg = Image()
            depth_msg.header = header
            depth_msg.height, depth_msg.width = result.depth.shape
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = False
            depth_msg.step = result.depth.shape[1] * 4
            depth_msg.data = result.depth.astype(np.float32).tobytes()
            self.depth_pub.publish(depth_msg)
        # 2. Segmentation mask (mono8)
        mask_u8 = (result.mask.astype(np.uint8) * 255)
        seg_msg = Image()
        seg_msg.header = header
        seg_msg.height, seg_msg.width = mask_u8.shape
        seg_msg.encoding = "mono8"
        seg_msg.is_bigendian = False
        seg_msg.step = mask_u8.shape[1]
        seg_msg.data = mask_u8.tobytes()
        self.seg_pub.publish(seg_msg)
        # 3. Overlay (rgb8)
        overlay = rgb.copy()
        overlay[result.mask] = (
            overlay[result.mask].astype(np.float32) * 0.5 +
            np.array([100, 100, 255], dtype=np.float32) * 0.5
        ).astype(np.uint8)
        overlay_msg = self._numpy_to_image_msg(overlay, "rgb8", header.frame_id)
        overlay_msg.header = header
        self.overlay_pub.publish(overlay_msg)

        # 4. Traversable path point cloud
        if self.publish_pc and result.depth is not None:
            pc_msg = self._make_traversable_cloud(
                result.depth, rgb, result.mask, header)
            self.pc_pub.publish(pc_msg)

    # ── Point cloud from depth + mask ───────────────────────────────────
    def _make_traversable_cloud(self, depth: np.ndarray, rgb: np.ndarray,
                                 mask: np.ndarray, header: Header) -> PointCloud2:

        H, W = depth.shape
        ds = self.pc_downsample
        depth_ds = depth[::ds, ::ds]
        mask_ds = mask[::ds, ::ds]
        rgb_ds = rgb[::ds, ::ds]
        Hd, Wd = depth_ds.shape
        u, v = np.meshgrid(
            np.arange(Wd) * ds,
            np.arange(Hd) * ds,
        )
        valid = (
            mask_ds &
            (depth_ds > self.pc_min_depth) &
            (depth_ds < self.pc_max_depth) &
            np.isfinite(depth_ds)
        )
        Z = depth_ds[valid].astype(np.float32)
        U = u[valid].astype(np.float32)
        V = v[valid].astype(np.float32)
        X_cam = (U - self.cx) * Z / self.fx
        Y_cam = (V - self.cy) * Z / self.fy
        Z_cam = Z
        pts_cam = np.stack([X_cam, Y_cam, Z_cam], axis=-1)  # (N, 3)
        if self.transform_to_base and self.R_cam_to_base is not None:
            pts_base = (self.R_cam_to_base @ pts_cam.T).T + self.t_cam_to_base
        else:
            pts_base = pts_cam

        colors = rgb_ds[valid]  # (N, 3) uint8
        N = len(pts_base)
        point_step = 16  # x, y, z, rgb_packed

        r = colors[:, 0].astype(np.uint32)
        g = colors[:, 1].astype(np.uint32)
        b = colors[:, 2].astype(np.uint32)
        rgb_packed = np.array(
            (r << 16) | (g << 8) | b, dtype=np.uint32
        ).view(np.float32)

        buf = np.zeros((N, 4), dtype=np.float32)
        buf[:, 0] = pts_base[:, 0]
        buf[:, 1] = pts_base[:, 1]
        buf[:, 2] = pts_base[:, 2]
        buf[:, 3] = rgb_packed

        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = self.pc_frame
        msg.height = 1
        msg.width = N
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * N
        msg.data = buf.tobytes()
        msg.is_dense = True

        return msg

    def _numpy_to_image_msg(self, rgb: np.ndarray, encoding: str,
                            frame_id: str) -> Image:
        msg = Image()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = frame_id
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = rgb.shape[1] * rgb.shape[2] if rgb.ndim == 3 else rgb.shape[1]
        msg.data = rgb.tobytes()
        return msg

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
