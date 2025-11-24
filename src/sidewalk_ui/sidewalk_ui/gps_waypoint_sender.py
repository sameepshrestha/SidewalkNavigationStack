#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import NavSatFix
import math

class GpsGoalNode(Node):
    def __init__(self):
        super().__init__("gps_goal_node")

        self.datum_lat = None
        self.datum_lon = None
        self.datum_set = False

        self.sub_gui = self.create_subscription(
            Point, # x=Lat, y=Lon
            '/gui/set_gps_goal',
            self.goal_callback,
            10
        )

        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/gps/data', # Listens to the Bridge Node
            self.gps_callback,
            10
        )

        self.pub_nav_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.get_logger().info("Waiting for robot GPS fix to establish Map Origin...")

    def gps_callback(self, msg):
        if not self.datum_set and msg.status.status >= -1: 
            self.datum_lat = msg.latitude
            self.datum_lon = msg.longitude
            self.datum_set = True
            self.get_logger().info(f"Map Origin Auto-Set to: {self.datum_lat}, {self.datum_lon}")
            

    def goal_callback(self, msg):
        if not self.datum_set:
            self.get_logger().warn("Cannot set goal yet! Robot hasn't found GPS Origin.")
            return

        target_lat = msg.x
        target_lon = msg.y
        
        self.get_logger().info(f"Calculating path to: {target_lat}, {target_lon}")
        delta_lat = target_lat - self.datum_lat
        delta_lon = target_lon - self.datum_lon
        
        map_y = delta_lat * 111132.0
        
        map_x = delta_lon * 111132.0 * math.cos(math.radians(self.datum_lat))

        nav_msg = PoseStamped()
        nav_msg.header.stamp = self.get_clock().now().to_msg()
        nav_msg.header.frame_id = "map" # Goal is an absolute coordinate on the Map

        nav_msg.pose.position.x = map_x
        nav_msg.pose.position.y = map_y
        nav_msg.pose.position.z = 0.0

        nav_msg.pose.orientation.w = 1.0 

        self.pub_nav_goal.publish(nav_msg)
        self.get_logger().info(f"Goal Published (Map Frame): X={map_x:.2f}m, Y={map_y:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = GpsGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()