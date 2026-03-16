#!/usr/bin/env python3
"""
FrodoBots Traversability Map Node
==================================
Converts the traversable-path PointCloud2 from frodobot_perception_ros into
a rolling 2D OccupancyGrid centred on the robot.

How it works:
  1. Subscribes to /perception/traversable_cloud (PointCloud2, in base_link frame)
  2. For every incoming cloud, marks grid cells containing traversable points as FREE (0)
  3. Cells that have never been seen stay UNKNOWN (-1)
  4. Publishes /mapping/traversability (nav_msgs/OccupancyGrid) at a fixed rate

Grid convention (Nav2 standard):
  -1  = unknown (never observed)
   0  = free / traversable
  100 = lethal obstacle

Future extensions (see config/mapping.yaml):
  - slope_filter: mark cells as obstacle if height variance within the cell > threshold
  - decay: cells fade back to unknown after N seconds (robot moves, old data stales)
"""
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class TraversabilityMapNode(Node):
    def __init__(self):
        super().__init__("traversability_map_node")

        self.declare_parameter("resolution", 0.05)      # metres per cell
        self.declare_parameter("grid_width_m", 20.0)    # total grid width  (metres)
        self.declare_parameter("grid_height_m", 20.0)   # total grid height (metres)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_id", "base_link") # grid always centred here
        self.declare_parameter("cloud_topic", "/perception/traversable_cloud")
        self.declare_parameter("output_topic", "/mapping/traversability")
        self.declare_parameter("decay_s", 0.0)
        self.declare_parameter("slope_filter_enable", False)
        self.declare_parameter("slope_height_thresh_m", 0.25)
        self.declare_parameter("footprint_free_radius_m", 0.6)
        self.declare_parameter("cloud_x_offset_m", 0.0)
        self.declare_parameter("obstacle_x_min_m", 1.5)
        self.declare_parameter("obstacle_x_max_m", 10.0)
        self.declare_parameter("obstacle_width_half_m", 5.0)
        self.obstacle_x_min_m = self.get_parameter("obstacle_x_min_m").value
        self.obstacle_x_max_m = self.get_parameter("obstacle_x_max_m").value
        self.obstacle_width_half_m = self.get_parameter("obstacle_width_half_m").value
        res      = self.get_parameter("resolution").value
        w_m      = self.get_parameter("grid_width_m").value
        h_m      = self.get_parameter("grid_height_m").value
        rate_hz  = self.get_parameter("publish_rate_hz").value
        self.frame_id   = self.get_parameter("frame_id").value
        cloud_topic     = self.get_parameter("cloud_topic").value
        output_topic    = self.get_parameter("output_topic").value
        self.decay_s    = self.get_parameter("decay_s").value
        self.slope_en   = self.get_parameter("slope_filter_enable").value
        self.slope_thr  = self.get_parameter("slope_height_thresh_m").value
        footprint_r     = self.get_parameter("footprint_free_radius_m").value
        self.x_offset   = self.get_parameter("cloud_x_offset_m").value
        cx = int((0.0 - (-w_m / 2.0)) / res)  # grid index of robot x=0
        cy = int((0.0 - (-h_m / 2.0)) / res)  # grid index of robot y=0
        r_cells = int(math.ceil(footprint_r / res))
        ys, xs = np.ogrid[-cy:int(math.ceil(h_m/res))-cy,
                          -cx:int(math.ceil(w_m/res))-cx]
        self._footprint_mask = (xs**2 + ys**2) <= r_cells**2
        self.res = res
        self.grid_w = int(math.ceil(w_m / res))   # cells
        self.grid_h = int(math.ceil(h_m / res))   # cells
        self.origin_x = -w_m / 2.0                # metres from robot to grid origin (bottom-left)
        self.origin_y = -h_m / 2.0

        xs = self.origin_x +np.arange(self.grid_w)*self.res
        ys = self.origin_y +np.arange(self.grid_h)*self.res
        X,Y = np.meshgrid(xs,ys)
        self._obstacle_mask  = (X>self.obstacle_x_min_m )&(X<self.obstacle_x_max_m)&(np.abs(Y)<self.obstacle_width_half_m)
        self.cost  = np.full((self.grid_h, self.grid_w), -1, dtype=np.int8)
        self.stamp = np.zeros((self.grid_h, self.grid_w), dtype=np.float64)  # last-seen time
        # For slope filter: track min/max Z per cell in the current sweep
        self._zmin = np.full((self.grid_h, self.grid_w),  1e9, dtype=np.float32)
        self._zmax = np.full((self.grid_h, self.grid_w), -1e9, dtype=np.float32)

        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, output_topic, latching)

        self.create_subscription(
            PointCloud2, cloud_topic, self._cloud_cb, 10
        )

        self.create_timer(1.0 / rate_hz, self._publish_map)

        self.get_logger().info(
            f"Traversability map: {self.grid_w}×{self.grid_h} cells "
            f"@ {res*100:.0f}cm/cell  "
            f"({w_m}×{h_m}m window centred on {self.frame_id})"
        )
    def _cloud_cb(self, msg: PointCloud2):
        now = time.time()
        self.cost.fill(-1)
        self.cost[self._obstacle_mask] = 100 #new obstacle map added where the traversal region was not found, only in visible reigion
        self.stamp.fill(0.0)
        if self.decay_s > 0:
            stale = (self.cost == 0) & ((now - self.stamp) > self.decay_s)
            self.cost[stale]  = -1  # expired free cell → unknown
            self.stamp[stale] = 0.0
        if self.slope_en:
            self._zmin[:] =  1e9
            self._zmax[:] = -1e9
        fields = ["x", "y", "z"] if self.slope_en else ["x", "y"]
        try:
            pts = np.array(list(pc2.read_points(msg, field_names=fields, skip_nans=True)))
        except Exception:
            return

        if pts.size == 0:
            return
        x = pts["x"] if pts.dtype.names else pts[:, 0]
        y = pts["y"] if pts.dtype.names else pts[:, 1]
        x = x + self.x_offset
        ix = np.floor((x - self.origin_x) / self.res).astype(np.int32)
        iy = np.floor((y - self.origin_y) / self.res).astype(np.int32)
        valid = (ix >= 0) & (ix < self.grid_w) & (iy >= 0) & (iy < self.grid_h)
        ix, iy = ix[valid], iy[valid]
        if self.slope_en:
            z = (pts["z"] if pts.dtype.names else pts[:, 2])[valid]
            np.minimum.at(self._zmin, (iy, ix), z)
            np.maximum.at(self._zmax, (iy, ix), z)
        # Morphological Dilation: Fill holes in sparse point cloud
        frame_mask = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8)
        frame_mask[iy, ix] = 1
        # Use an elliptical kernel (e.g., 5x5 cells = 25cm diameter if res is 0.05)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated_mask = cv2.dilate(frame_mask, kernel, iterations=1)
        free_y, free_x = np.where(dilated_mask == 1)
        self.cost[free_y, free_x] = 0
        self.stamp[free_y, free_x] = now
        if self.slope_en:
            height_range = self._zmax - self._zmin
            valid_slope  = height_range < 1e8  # cells that actually got points
            obstacle     = valid_slope & (height_range > self.slope_thr)
            # Back-fill over any previously marked free cells in this sweep
            self.cost[obstacle]  = 100
            self.stamp[obstacle] = now

    def _stamp_footprint(self):
        now = time.time()
        self.cost[self._footprint_mask]  = 0
        self.stamp[self._footprint_mask] = now

    def _publish_map(self):
        self._stamp_footprint()  # always clear the near-field blind zone
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.info = MapMetaData()
        msg.info.resolution = self.res
        msg.info.width  = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # OccupancyGrid data is row-major, int8: -1=unknown, 0=free, 100=lethal
        msg.data = self.cost.flatten().tolist()

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
