#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy # <--- Import this
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum

class DatumAverager(Node):
    def __init__(self):
        super().__init__('datum_averager_node')
        self.target_sample_count = 10
        self.samples = []
        self.datum_set = False

        self.subscription = self.create_subscription(
            NavSatFix, '/gps/data', self.gps_callback, 10)

        self.datum_client = self.create_client(SetDatum, '/datum')

        # --- THE FIX: Create a LATCHED (Transient Local) Publisher ---
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # NOTE: We do NOT put '10' here. We put the qos object directly.
        self.datum_pub = self.create_publisher(NavSatFix, '/verified_datum', latching_qos)

        self.get_logger().info("Datum Averager Started: Waiting for GPS data...")

    def gps_callback(self, msg):
        if self.datum_set: return

        # Ignore invalid zeros
        if abs(msg.latitude) < 0.1 and abs(msg.longitude) < 0.1: return

        self.samples.append(msg)
        self.get_logger().info(f"Collecting GPS sample {len(self.samples)}/{self.target_sample_count}")

        if len(self.samples) >= self.target_sample_count:
            self.set_datum_from_average()

    def set_datum_from_average(self):
        avg_lat = sum(msg.latitude for msg in self.samples) / len(self.samples)
        avg_lon = sum(msg.longitude for msg in self.samples) / len(self.samples)
        avg_alt = sum(msg.altitude for msg in self.samples) / len(self.samples)

        self.get_logger().info(f"Calculated Average: Lat: {avg_lat}, Lon: {avg_lon}")

        # 1. Call EKF Service (Navsat Transform)
        if self.datum_client.wait_for_service(timeout_sec=1.0):
            request = SetDatum.Request()
            request.geo_pose.position.latitude = avg_lat
            request.geo_pose.position.longitude = avg_lon
            request.geo_pose.position.altitude = avg_alt
            request.geo_pose.orientation.w = 1.0
            
            future = self.datum_client.call_async(request)
            future.add_done_callback(self.service_response_callback)
        else:
            self.get_logger().warn("Datum service not available! Proceeding anyway...")

        # 2. Publish to OSM Planner (LATCHED)
        msg = NavSatFix()
        msg.latitude = avg_lat
        msg.longitude = avg_lon
        msg.altitude = avg_alt
        
        self.datum_pub.publish(msg) 
        self.get_logger().info("PUBLISHED VERIFIED DATUM (Latched)")
        
        self.datum_set = True

    def service_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info("SUCCESS: EKF Datum set.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DatumAverager())
    rclpy.shutdown()

if __name__ == '__main__':
    main()