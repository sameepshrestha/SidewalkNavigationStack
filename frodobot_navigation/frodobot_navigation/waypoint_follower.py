#!/usr/bin/env python3
"""
FrodoBots Direct GPS Waypoint Follower
======================================
Loads sparse GPS goals from route.yaml.
Subscribes to global GPS and local robot heading to compute a RELATIVE 
goal heading vector. Publishes this relative vector (in base_link) 
for MPPI's DirectionCritic to follow.

Behavior:
  - Subscribes to /earth_rover/gps/fix (Current location)
  - Subscribes to /robot_heading (Current yaw in radians)
  - Computes relative heading: (Global Goal Angle - Robot Current Angle)
  - Publishes unit heading vector on /goal_heading in base_link frame.
  - Continuously sends a dummy 1km Path to Nav2's FollowPath server.

"""

import math
import os
import yaml
import folium
import sys

# Add Globalrouter to path for imports
# Assumes project root is parent of frodobot_navigation
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.append(os.path.join(ROOT_DIR, "Globalrouter", "planner_settings"))
sys.path.append(os.path.join(ROOT_DIR, "Globalrouter", "global_planner"))

try:
    from route_planner import RoutePlanner
    from config import PlannerConfig
except ImportError as e:
    # If not found via relative path, try absolute fallback
    sys.path.append("/home/sameep/phd_research/navigation_stack_compeition/Globalrouter/planner_settings")
    sys.path.append("/home/sameep/phd_research/navigation_stack_compeition/Globalrouter/global_planner")
    from route_planner import RoutePlanner
    from config import PlannerConfig

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Vector3Stamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from nav2_msgs.action import FollowPath as FollowPathAction
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor


EARTH_RADIUS_M = 6378137.0


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        self.declare_parameter("route_file", "")
        self.declare_parameter("goal_tolerance_m", 6.0)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("gps_topic", "/earth_rover/gps/fix")
        self.declare_parameter("heading_topic", "/robot_heading")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("robot_frame", "base_link") 
        self.declare_parameter("waypoint_spacing_m", 20.0)
       
        route_file = self.get_parameter("route_file").value
        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        gps_topic = self.get_parameter("gps_topic").value
        heading_topic = self.get_parameter("heading_topic").value
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.wp_spacing = self.get_parameter("waypoint_spacing_m").value

        self.goals_latlon = []      # [(lat, lon), ...]
        self.current_idx = 0

        self.robot_lat = None
        self.robot_lon = None
        self.robot_heading_rad = 0.0 # Stores live heading from SimpleHeadingNode

        self._follow_goal_handle = None
        self._action_sent = False

        if not route_file or not os.path.isfile(route_file):
            self.get_logger().error(f"route_file not found: {route_file!r}")
            self.get_logger().error("Pass --ros-args -p route_file:=/path/to/route.yaml")
            return

        with open(route_file, "r") as f:
            cfg = yaml.safe_load(f)

        # Allow adjusting spacing from YAML, otherwise use ROS parameter
        self.wp_spacing = cfg.get("waypoint_spacing_m", self.wp_spacing)

        goals = cfg.get("goals", [])
        if not goals:
            self.get_logger().error("No goals found in route file.")
            return

        def to_lonlat(pt):
            if isinstance(pt, dict):
                return [float(pt["lon"]), float(pt["lat"])]
            return [float(pt[0]), float(pt[1])]

        start_cfg = cfg.get("start")
        if not start_cfg:
            self.get_logger().error("No 'start' coordinate in route file. Needed for Valhalla.")
            return

        start_pt = to_lonlat(start_cfg)
        goals_pts = [to_lonlat(g) for g in goals]

        self.get_logger().info(f"Generating Valhalla route (spacing={self.wp_spacing}m)...")
        try:
            planner_cfg = PlannerConfig(waypoint_spacing_m=self.wp_spacing)
            planner = RoutePlanner(config=planner_cfg)
            self.goals_latlon = planner.generate_route(start=start_pt, goals=goals_pts)
            self.get_logger().info(f"Route Planner generated {len(self.goals_latlon)} waypoints.")
        except Exception as e:
            self.get_logger().error(f"Route Planner failed: {e}. Falling back to straight line.")
            self.goals_latlon = [(p[1], p[0]) for p in goals_pts]

        self._save_folium_map()

        cb_group = ReentrantCallbackGroup()

        self._follow_client = ActionClient(
            self, FollowPathAction, "follow_path",
            callback_group=cb_group)

        self.heading_pub = self.create_publisher(Vector3Stamped, "/goal_heading", 5)

        # GPS Sub
        self.create_subscription(
            NavSatFix, gps_topic, self._gps_cb, qos_profile_sensor_data, callback_group=cb_group
        )
        
        # Heading Sub
        self.create_subscription(
            Float64, heading_topic, self._heading_cb, 10, callback_group=cb_group
        )

        self.create_timer(1.0 / publish_rate_hz, self._timer_cb, callback_group=cb_group)

        self.get_logger().info("Relative GPS Waypoint Follower ready — waiting for GPS fix")

    def _gps_cb(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or msg.status.status < 0:
            return
        self.robot_lat = msg.latitude
        self.robot_lon = msg.longitude

    def _heading_cb(self, msg: Float64):
        self.robot_heading_rad = msg.data

    def _timer_cb(self):
        if self.robot_lat is None or self.robot_lon is None:
            return

        if self.current_idx >= len(self.goals_latlon):
            self.get_logger().info("All waypoints completed.", throttle_duration_sec=10.0)
            if self._action_sent:
                self._cancel_follow_path()
                self._action_sent = False
            return
        if not self._action_sent:
            self._send_dummy_path_goal()
            self._action_sent = True

        goal_lat, goal_lon = self.goals_latlon[self.current_idx]
        
        lat_rad = math.radians(self.robot_lat)
        lon_rad = math.radians(self.robot_lon)
        glat_rad = math.radians(goal_lat)
        glon_rad = math.radians(goal_lon)

        dx = (glon_rad - lon_rad) * math.cos(lat_rad) * EARTH_RADIUS_M
        dy = (glat_rad - lat_rad) * EARTH_RADIUS_M
        dist = math.hypot(dx, dy)
        while dist < self.goal_tolerance:
            self.get_logger().info(
                f"Waypoint {self.current_idx + 1}/{len(self.goals_latlon)} reached "
                f"(dist={dist:.2f} m)"
            )
            self.current_idx += 1

            if self.current_idx >= len(self.goals_latlon):
                self.get_logger().info("Route complete!", throttle_duration_sec=5.0)
                if self._action_sent:
                    self._cancel_follow_path()
                    self._action_sent = False
                return
            goal_lat, goal_lon = self.goals_latlon[self.current_idx]
            glat_rad = math.radians(goal_lat)
            glon_rad = math.radians(goal_lon)
            dx = (glon_rad - lon_rad) * math.cos(lat_rad) * EARTH_RADIUS_M
            dy = (glat_rad - lat_rad) * EARTH_RADIUS_M
            dist = math.hypot(dx, dy)

        # North-referenced Clockwise Heading:
        # dx is Easting, dy is Northing. atan2(dx, dy) gives 0 at North, 90 at East.
        global_goal_heading = math.atan2(dx, dy)
        # relative_heading_cw is the angle from robot front to goal CW.
        # robot_heading_rad is already North=0, CW.
        relative_heading_cw = math.atan2(
            math.sin(global_goal_heading - self.robot_heading_rad),
            math.cos(global_goal_heading - self.robot_heading_rad)
        )

        self.get_logger().info(f"Relative CW heading: {math.degrees(relative_heading_cw):.1f}°")
        self.get_logger().info(f"Global heading (North=0, CW): {math.degrees(global_goal_heading):.1f}°")
        self.get_logger().info(f"Robot heading (North=0, CW): {math.degrees(self.robot_heading_rad):.1f}°")
        self.get_logger().info(f"Robot GPS: {self.robot_lat:.8f}, {self.robot_lon:.8f}")
        self.get_logger().info(f"Goal GPS: {goal_lat:.8f}, {goal_lon:.8f}")

        # Convert relative CW angle to base_link vector (X=front, Y=left)
        # If relative_heading_cw is +90 (Right), vector is (0, -1)
        # If relative_heading_cw is -90 (Left), vector is (0, 1)
        ux = math.cos(relative_heading_cw)
        uy = -math.sin(relative_heading_cw)

        self._publish_heading(ux, uy)

    def _publish_heading(self, ux: float, uy: float):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Publish in the ROBOT frame since the vector is now relative!
        msg.header.frame_id = self.robot_frame 
        msg.vector.x = ux
        msg.vector.y = uy
        msg.vector.z = 0.0
        self.heading_pub.publish(msg)

    def _save_folium_map(self):
        if not self.goals_latlon:
            return
        
        # Center on first waypoint
        center = self.goals_latlon[0]
        m = folium.Map(location=center, zoom_start=18)
        
        # Plot goals
        for i, (lat, lon) in enumerate(self.goals_latlon):
            color = "red" if i == len(self.goals_latlon) - 1 else "orange"
            folium.Marker(
                [lat, lon],
                popup=f"Waypoint {i+1}",
                icon=folium.Icon(color=color, icon="info-sign")
            ).add_to(m)
            
        # Draw path
        folium.PolyLine(self.goals_latlon, color="blue", weight=2.5, opacity=0.8).add_to(m)
        
        save_path = "current_route.html"
        m.save(save_path)
        self.get_logger().info(f"Saved Folium route map to {os.path.abspath(save_path)}")

    # ── Action Interface: Dummy Path ─────────────────────────────────────────
    def _send_dummy_path_goal(self):
        if not self._follow_client.server_is_ready():
            self.get_logger().warn("FollowPath server not ready! Can't start MPPI.", throttle_duration_sec=2.0)
            self._action_sent = False 
            return

        self.get_logger().info("Sending dummy path to wake up Nav2/MPPI...")
        
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.global_frame

        # Start at generic (0,0)
        p1 = PoseStamped()
        p1.header = path.header
        p1.pose.orientation.w = 1.0
        path.poses.append(p1)

        # Extend dummy distance
        p2 = PoseStamped()
        p2.header = path.header
        p2.pose.position.x = 1000.0
        p2.pose.orientation.w = 1.0
        path.poses.append(p2)

        goal_msg = FollowPathAction.Goal()
        goal_msg.path = path
        goal_msg.controller_id = "FollowPath"

        future = self._follow_client.send_goal_async(goal_msg)
        future.add_done_callback(self._follow_goal_response_cb)

    def _follow_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dummy Path goal rejected by Nav2!")
            self._action_sent = False
            return
        self._follow_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._follow_goal_result_cb)
        self.get_logger().info("Nav2 accepted dummy path. Directional control active.")

    def _cancel_follow_path(self):
        if self._follow_goal_handle:
            self.get_logger().info("Route complete. Cancelling Nav2 goal.")
            self._follow_goal_handle.cancel_goal_async()
            self._follow_goal_handle = None

    def _follow_goal_result_cb(self, future):
        result = future.result().status
        self.get_logger().warn(f"Nav2 Dummy Path Action ended with status: {result}")
        self._action_sent = False
        self._follow_goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()