#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import osmnx as ox
import networkx as nx
import numpy as np
import os
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry # Change from NavSatFix (for robot pos)
class OsmCarrotPlanner(Node):
    def __init__(self):
        super().__init__('osm_carrot_planner')

        # STATE
        self.datum_lat = None
        self.datum_lon = None
        self.G = None             
        self.path_xy = None       
        self.last_idx = 0
        self.map_ready = False
        
        self.declare_parameter('lookahead', 5.0)
        self.lookahead = self.get_parameter('lookahead').value

        qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavSatFix, '/verified_datum', self.datum_callback, qos)

        self.create_subscription(NavSatFix, '/gui/set_gps_goal', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odometry/global', self.robot_pos_callback, 10)
        self.carrot_pub = self.create_publisher(PoseStamped, '/local_goal', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/path_viz', 10)
        self.get_logger().info("STATE: Waiting for Datum...")

    def datum_callback(self, msg):
        if self.map_ready: return 
        
        self.datum_lat = msg.latitude
        self.datum_lon = msg.longitude
        self.get_logger().info(f"Datum Received: {self.datum_lat:.6f}, {self.datum_lon:.6f}")

        map_filename = f"sidewalk_map_{self.datum_lat:.3f}_{self.datum_lon:.3f}.graphml"

        try:
            if os.path.exists(map_filename):
                self.get_logger().info(f"Loading cached map: {map_filename}")
                self.G = ox.load_graphml(map_filename)
            else:
                self.get_logger().info("Downloading from OSM (Internet required)...")
                self.G = ox.graph_from_point(
                    (self.datum_lat, self.datum_lon), 
                    dist=800, 
                    network_type='walk', 
                    simplify=True
                )
                ox.save_graphml(self.G, map_filename)
            
            self.map_ready = True
            self.get_logger().info("STATE: Map Ready! Waiting for Goal topic /ui_goal...")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")

    def goal_callback(self, msg):
        if not self.map_ready:
            self.get_logger().warn("Goal received but Map not ready (Check Datum)")
            return

        self.get_logger().info(f"Planning Path to: {msg.latitude}, {msg.longitude}")

        orig_node = ox.distance.nearest_nodes(self.G, self.datum_lon, self.datum_lat)
        dest_node = ox.distance.nearest_nodes(self.G, msg.longitude, msg.latitude)

        try:
            node_ids = nx.shortest_path(self.G, orig_node, dest_node, weight='length')
            
            path_list = []
            for nid in node_ids:
                node = self.G.nodes[nid]
                x, y = self.geo_to_meter(node['y'], node['x'])
                path_list.append([x, y])
            
            self.path_xy = np.array(path_list)
            self.last_idx = 0 
            
            self.get_logger().info(f"STATE: Path Found ({len(self.path_xy)} nodes). Following Carrot...")
            self.publish_static_path_viz()
            
        except nx.NetworkXNoPath:
            self.get_logger().error("Pathfinding failed: No valid sidewalk path found.")

    def robot_pos_callback(self, msg):
        if self.path_xy is None: return 

        robot_xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        
        carrot_xy, is_turning = self.compute_carrot_and_turn(robot_xy)
    # ... rest of function is same ...
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = carrot_xy[0]
        pose_msg.pose.position.y = carrot_xy[1]
        self.carrot_pub.publish(pose_msg)
        
        self.publish_carrot_viz(carrot_xy, is_turning)

    def compute_carrot_and_turn(self, robot_xy):
        L = self.lookahead
        search_range = 20
        start_i = max(0, self.last_idx - 5)
        end_i = min(len(self.path_xy) - 1, self.last_idx + search_range)
        
        best_proj = None
        min_dist = float('inf')
        best_idx = self.last_idx

        for i in range(start_i, end_i):
            p1 = self.path_xy[i]
            p2 = self.path_xy[i+1]
            proj = self.project_point(p1, p2, robot_xy)
            d = np.linalg.norm(robot_xy - proj)
            if d < min_dist:
                min_dist = d; best_proj = proj; best_idx = i
        
        self.last_idx = best_idx
        
        # B. Detect Sharp Turn
        is_turning = False
        if best_idx < len(self.path_xy) - 2:
            v1 = self.path_xy[best_idx+1] - self.path_xy[best_idx]
            v2 = self.path_xy[best_idx+2] - self.path_xy[best_idx+1]
            cos_a = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
            angle = np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0)))
            if angle > 35: # >35 deg is a sharp turn
                L = 1.5
                is_turning = True

        # C. Advance L meters
        rem = L
        curr = best_proj
        idx = best_idx
        while idx < len(self.path_xy) - 1:
            p_next = self.path_xy[idx+1]
            dist = np.linalg.norm(p_next - curr)
            if rem <= dist:
                vec = (p_next - curr) / dist
                return curr + (vec * rem), is_turning
            rem -= dist
            curr = p_next
            idx += 1
            
        return self.path_xy[-1], is_turning

    def project_point(self, p1, p2, p):
        line = p2 - p1
        if np.all(line==0): return p1
        t = np.dot(p - p1, line) / np.dot(line, line)
        return p1 + line * np.clip(t, 0.0, 1.0)

    def geo_to_meter(self, lat, lon):
        R = 6371000.0
        x = np.radians(lon - self.datum_lon) * np.cos(np.radians(self.datum_lat)) * R
        y = np.radians(lat - self.datum_lat) * R
        return x, y

    def publish_static_path_viz(self):
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"; m.id = 0; m.type = Marker.LINE_STRIP; m.scale.x = 0.3
        m.color.a = 1.0; m.color.g = 1.0; m.pose.orientation.w = 1.0
        for pt in self.path_xy:
            p = Point(x=pt[0], y=pt[1], z=0.0)
            m.points.append(p)
        ma.markers.append(m)
        self.viz_pub.publish(ma)

    def publish_carrot_viz(self, xy, is_turning):
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"; m.id = 1; m.type = Marker.SPHERE
        m.scale.x = 0.6; m.scale.y = 0.6; m.scale.z = 0.6
        m.color.a = 1.0; m.color.r = 1.0; m.pose.orientation.w = 1.0
        if is_turning: m.color.b = 1.0; m.color.r = 0.0
        m.pose.position.x = xy[0]; m.pose.position.y = xy[1]
        ma.markers.append(m)
        self.viz_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OsmCarrotPlanner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()