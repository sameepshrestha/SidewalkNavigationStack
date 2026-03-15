#!/usr/bin/env python3
"""
Quick gyro bias calibration.
Leave the robot COMPLETELY STILL, then run this for ~10 seconds.
It will print the bias values to paste into bridge_params.yaml.

Usage:
  ros2 topic echo /earth_rover/imu/data_raw --once  # verify data is flowing
  python3 calibrate_gyro.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np


class GyroCal(Node):
    def __init__(self):
        super().__init__("gyro_cal")
        self.samples = []
        self.create_subscription(Imu, "/earth_rover/imu/data_raw", self.cb, 10)
        self.create_timer(10.0, self.done)
        self.get_logger().info("Collecting gyro samples for 10s — keep robot STILL...")

    def cb(self, msg):
        self.samples.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

    def done(self):
        if not self.samples:
            self.get_logger().error("No IMU data received!")
            raise SystemExit(1)
        arr = np.array(self.samples)
        mean = arr.mean(axis=0)
        std = arr.std(axis=0)
        self.get_logger().info(f"Collected {len(self.samples)} samples")
        self.get_logger().info(f"Mean gyro (rad/s): [{mean[0]:.6f}, {mean[1]:.6f}, {mean[2]:.6f}]")
        self.get_logger().info(f"Std  gyro (rad/s): [{std[0]:.6f}, {std[1]:.6f}, {std[2]:.6f}]")
        self.get_logger().info("")
        self.get_logger().info("If the mean Z value here is NOT close to zero,")
        self.get_logger().info("your gyro bias is wrong. The bridge should produce")
        self.get_logger().info("near-zero values after bias subtraction.")
        self.get_logger().info("")
        self.get_logger().info("NOTE: These values are AFTER the bridge has already applied bias.")
        self.get_logger().info("To fix, ADD these residuals to your current gyro_bias_deg_s in bridge_params.yaml")
        # Convert residual from rad/s back to deg/s for the config
        residual_deg = mean * (180.0 / np.pi)
        self.get_logger().info(f"Residual bias (deg/s): [{residual_deg[0]:.4f}, {residual_deg[1]:.4f}, {residual_deg[2]:.4f}]")
        raise SystemExit(0)


def main():
    rclpy.init()
    node = GyroCal()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
