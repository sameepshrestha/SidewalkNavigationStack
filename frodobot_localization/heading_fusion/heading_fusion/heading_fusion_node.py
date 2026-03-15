#!/usr/bin/env python3
"""
Simple GPS + Magnetometer Heading Node for ROS 2

Behavior:
- Initialize heading once from magnetometer.
- When GPS motion is reliable, update heading from GPS course-over-ground.
- When stationary / slow, hold the last heading.
- Publishes only robot heading.

Published topics:
- /robot_heading      (std_msgs/msg/Float64)  heading in radians
- /robot_heading_deg  (std_msgs/msg/Float64)  heading in degrees

Heading convention:
- ENU/world-style heading
- 0 rad   = East
- +pi/2   = North
- atan2(dy, dx) convention
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, NavSatFix
from std_msgs.msg import Float64


EARTH_RADIUS_M = 6378137.0


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    return wrap_angle(a - b)


@dataclass
class LocalXY:
    x: float
    y: float
    stamp_sec: float


class SimpleHeadingNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_heading_node")
        # Topics
        self.declare_parameter("mag_topic", "/earth_rover/imu/mag")
        self.declare_parameter("gps_topic", "/earth_rover/gps/fix")
        self.declare_parameter("heading_topic", "/robot_heading")
        self.declare_parameter("heading_deg_topic", "/robot_heading_deg")
        # Magnetometer init
        self.declare_parameter("mag_offset_x", 0.0)
        self.declare_parameter("mag_offset_y", 0.0)
        self.declare_parameter("mag_offset_z", 0.0)
        self.declare_parameter("mag_init_mode", "xz_negz_x")
        # options:
        #   xz_negz_x     -> atan2(-mz,  mx)
        #   xz_z_x        -> atan2( mz,  mx)
        #   xz_negz_negx  -> atan2(-mz, -mx)
        #   xz_z_negx     -> atan2( mz, -mx)

        self.declare_parameter("yaw_mount_offset_deg", 0.0)

        # GPS heading
        self.declare_parameter("use_gps_heading", True)
        self.declare_parameter("gps_window_size", 7)
        self.declare_parameter("gps_min_distance_m", 1.5)
        self.declare_parameter("gps_min_speed_mps", 0.25)
        self.declare_parameter("gps_heading_alpha", 0.35)

        # Diagnostics
        self.declare_parameter("log_diagnostics_rate_hz", 2.0)

        self.mag_topic = self.get_parameter("mag_topic").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.heading_topic = self.get_parameter("heading_topic").value
        self.heading_deg_topic = self.get_parameter("heading_deg_topic").value

        self.mag_offset = np.array([
            float(self.get_parameter("mag_offset_x").value),
            float(self.get_parameter("mag_offset_y").value),
            float(self.get_parameter("mag_offset_z").value),
        ], dtype=float)

        self.mag_init_mode = str(self.get_parameter("mag_init_mode").value)
        self.yaw_mount_offset = math.radians(
            float(self.get_parameter("yaw_mount_offset_deg").value)
        )

        self.use_gps_heading = bool(self.get_parameter("use_gps_heading").value)
        self.gps_window_size = int(self.get_parameter("gps_window_size").value)
        self.gps_min_distance_m = float(self.get_parameter("gps_min_distance_m").value)
        self.gps_min_speed_mps = float(self.get_parameter("gps_min_speed_mps").value)
        self.gps_heading_alpha = float(self.get_parameter("gps_heading_alpha").value)
        self.log_hz = float(self.get_parameter("log_diagnostics_rate_hz").value)

        # State
        self.heading_est: float = 0.0
        self.is_initialized: bool = False
        self.last_mag_raw: Optional[np.ndarray] = None
        self.last_gps_heading: Optional[float] = None

        self.datum_lat_deg = float("nan")
        self.datum_lon_deg = float("nan")
        self.datum_is_set = False
        self.gps_points: Deque[LocalXY] = deque(maxlen=self.gps_window_size)

        # ROS
        self.mag_sub = self.create_subscription(
            MagneticField, self.mag_topic, self.mag_callback, 20
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, 20
        )

        self.heading_pub = self.create_publisher(Float64, self.heading_topic, 20)
        self.heading_deg_pub = self.create_publisher(Float64, self.heading_deg_topic, 20)

        if self.log_hz > 0.0:
            self.create_timer(1.0 / self.log_hz, self.log_diagnostics)

        self.get_logger().info("SimpleHeadingNode started")
        self.get_logger().info(f"  Mag topic: {self.mag_topic}")
        self.get_logger().info(f"  GPS topic: {self.gps_topic}")
        self.get_logger().info(f"  Heading topic: {self.heading_topic}")
        self.get_logger().info(f"  mag_init_mode: {self.mag_init_mode}")
        self.get_logger().info(
            f"  yaw_mount_offset_deg: {math.degrees(self.yaw_mount_offset):.1f}"
        )

    @staticmethod
    def to_sec(stamp) -> float:
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def latlon_to_local_xy(self, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
        lat0 = math.radians(self.datum_lat_deg)
        lon0 = math.radians(self.datum_lon_deg)
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)

        x = (lon - lon0) * math.cos(lat0) * EARTH_RADIUS_M
        y = (lat - lat0) * EARTH_RADIUS_M
        return x, y

    def mag_to_heading(self, m: np.ndarray) -> float:
        mx, my, mz = m

        if self.mag_init_mode == "xz_negz_x":
            yaw = math.atan2(-mz, mx)
        elif self.mag_init_mode == "xz_z_x":
            yaw = math.atan2(mz, mx)
        elif self.mag_init_mode == "xz_negz_negx":
            yaw = math.atan2(-mz, -mx)
        elif self.mag_init_mode == "xz_z_negx":
            yaw = math.atan2(mz, -mx)
        else:
            yaw = math.atan2(-mz, mx)

        return wrap_angle(yaw + self.yaw_mount_offset)

    def compute_window_heading(self) -> Optional[float]:
        if len(self.gps_points) < 3:
            return None

        points = list(self.gps_points)
        start, end = points[0], points[-1]

        dx_total = end.x - start.x
        dy_total = end.y - start.y
        distance = math.hypot(dx_total, dy_total)
        dt = max(1e-6, end.stamp_sec - start.stamp_sec)

        if distance < self.gps_min_distance_m:
            return None
        if (distance / dt) < self.gps_min_speed_mps:
            return None

        mean_x = sum(p.x for p in points) / len(points)
        mean_y = sum(p.y for p in points) / len(points)
        sxx = sum((p.x - mean_x) ** 2 for p in points)
        syy = sum((p.y - mean_y) ** 2 for p in points)
        sxy = sum((p.x - mean_x) * (p.y - mean_y) for p in points)

        heading = 0.5 * math.atan2(2.0 * sxy, sxx - syy)

        vx, vy = math.cos(heading), math.sin(heading)
        if vx * dx_total + vy * dy_total < 0.0:
            heading = wrap_angle(heading + math.pi)

        return heading

    def publish_heading(self) -> None:
        if not self.is_initialized:
            return

        msg_rad = Float64()
        msg_rad.data = self.heading_est
        self.heading_pub.publish(msg_rad)

        msg_deg = Float64()
        msg_deg.data = math.degrees(self.heading_est)
        self.heading_deg_pub.publish(msg_deg)

    def mag_callback(self, msg: MagneticField) -> None:
        if math.isnan(msg.magnetic_field.x):
            return

        self.last_mag_raw = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ], dtype=float)

        if not self.is_initialized:
            m = self.last_mag_raw - self.mag_offset
            self.heading_est = self.mag_to_heading(m)
            self.is_initialized = True
            self.get_logger().info(
                f"Initialized heading from magnetometer: {math.degrees(self.heading_est):.1f}°"
            )
            self.publish_heading()

    def gps_callback(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or msg.status.status < 0:
            return

        lat = float(msg.latitude)
        lon = float(msg.longitude)

        if not self.datum_is_set:
            self.datum_lat_deg = lat
            self.datum_lon_deg = lon
            self.datum_is_set = True
            self.get_logger().info(
                f"Set local GPS datum automatically: lat={lat:.8f}, lon={lon:.8f}"
            )

        x, y = self.latlon_to_local_xy(lat, lon)
        stamp = self.to_sec(msg.header.stamp)
        self.gps_points.append(LocalXY(x=x, y=y, stamp_sec=stamp))

        gps_heading = self.compute_window_heading()
        if gps_heading is None:
            return

        self.last_gps_heading = gps_heading

        if not self.is_initialized:
            self.heading_est = gps_heading
            self.is_initialized = True
            self.get_logger().info(
                f"Initialized heading from GPS course: {math.degrees(self.heading_est):.1f}°"
            )
        elif self.use_gps_heading:
            err = angle_diff(gps_heading, self.heading_est)
            self.heading_est = wrap_angle(
                self.heading_est + self.gps_heading_alpha * err
            )

        self.publish_heading()

    def log_diagnostics(self) -> None:
        if not self.is_initialized:
            return

        gps_str = "N/A" if self.last_gps_heading is None else f"{math.degrees(self.last_gps_heading):.1f}°"
        self.get_logger().info(
            f"Heading Diag | Heading Est: {math.degrees(self.heading_est):.1f}° | "
            f"GPS C.O.G: {gps_str}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()