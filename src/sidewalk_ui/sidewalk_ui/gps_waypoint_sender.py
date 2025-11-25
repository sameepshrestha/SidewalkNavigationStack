#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose

class DatumAverager(Node):
    def __init__(self):
        super().__init__('datum_averager_node')
        self.target_sample_count = 10
        self.samples = []
        self.datum_set = False

        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/data',
            self.gps_callback,
            10)

        # Create Client for the SetDatum service
        self.datum_client = self.create_client(SetDatum, '/datum')

        self.get_logger().info("Datum Averager Started: Waiting for GPS data...")

    def gps_callback(self, msg):
        # If datum is already set, stop processing to save CPU
        if self.datum_set:
            return

        self.samples.append(msg)
        self.get_logger().info(f"Collecting GPS sample {len(self.samples)}/{self.target_sample_count}")

        if len(self.samples) >= self.target_sample_count:
            self.set_datum_from_average()

    def set_datum_from_average(self):
        avg_lat = sum(msg.latitude for msg in self.samples) / len(self.samples)
        avg_lon = sum(msg.longitude for msg in self.samples) / len(self.samples)
        avg_alt = sum(msg.altitude for msg in self.samples) / len(self.samples)

        self.get_logger().info(f"Calculated Average: Lat: {avg_lat}, Lon: {avg_lon}")

        while not self.datum_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /datum service to be available...')

        request = SetDatum.Request()
        request.geo_pose.position.latitude = avg_lat
        request.geo_pose.position.longitude = avg_lon
        request.geo_pose.position.altitude = avg_alt
        
        # Orientation (Identity / Neutral)
        request.geo_pose.orientation.w = 1.0
        request.geo_pose.orientation.x = 0.0
        request.geo_pose.orientation.y = 0.0
        request.geo_pose.orientation.z = 0.0

        # 4. Call Service Async
        self.future = self.datum_client.call_async(request)
        self.future.add_done_callback(self.service_response_callback)
        
        # Mark as done so we don't try to set it again
        self.datum_set = True

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("SUCCESS: Datum service called successfully.")
            
            # IF YOU WANT THIS NODE TO STOP AFTER SETTING DATUM, UNCOMMENT BELOW:
            # import sys; sys.exit(0)
            
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DatumAverager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()