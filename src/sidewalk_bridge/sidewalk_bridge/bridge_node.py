#!/home/sameep/phd_research/sidewalkauto_ws/sidenv/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, BatteryState, NavSatFix
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np 
import math
from .robot_client import Robotclient
from .utilis import euler_to_quaternion 

SERVER_OFFER_URL = "http://100.102.181.1:8080/offer" 

class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")

        self.pub_front = self.create_publisher(Image, 'camera/front/image_raw',10)
        self.pub_left = self.create_publisher(Image, 'camera/left/image_raw',10)
        self.pub_right = self.create_publisher(Image, 'camera/right/image_raw',10)
    

        self.pub_imu = self.create_publisher(Imu, '/imu/data',10)
        self.pub_gps = self.create_publisher(NavSatFix,'/gps/data',10)
        self.sub_cmd = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.cv_bridge = CvBridge()
        self.get_logger().info(f"Inializing webrtc brige to (server)")
        try:
            self.client = Robotclient(SERVER_OFFER_URL )
            self.client.start()
            self.get_logger().info("Webrtc thread started")
        except Exception as e:
            self.get_logger().error("Failed to start webrtc client: {e}")
        self.timer = self.create_timer(0.1, self.update_loop)

    def update_loop(self):  # current time to sensor recorded data, need to change this 
        current_time = self.get_clock().now().to_msg()
        frame_front = self.client.front_camera_frame
        if frame_front is not None:
            self.publish_image(frame_front,"front_camera_link", self.pub_front, current_time)
        frame_left = self.client.left_camera_frame
        if frame_left is not None:
            self.publish_image(frame_left,"left_camera_link", self.pub_left, current_time)
        frame_right = self.client.right_camera_frame
        if frame_right is not None:
            self.publish_image(frame_right,"right_camera_link", self.pub_right, current_time)
        data = self.client.get_last_msg
        if data is not None:
            self.publish_imu(data, current_time)
        if hasattr(data, 'gps'):
            if data.gps.lat != 1000 and (abs(data.gps.lat) > 0.001 or abs(data.gps.lon) > 0.001):
                self.publish_raw_gps(data, current_time)
            else:
                pass

    def publish_raw_gps(self, data, current_time):
        nav_msg = NavSatFix()
        nav_msg.header.stamp = current_time
        nav_msg.header.frame_id = "gps_link" 
        nav_msg.latitude = data.gps.lat
        nav_msg.longitude = data.gps.lon
        nav_msg.altitude = getattr(data.gps, 'alt', 0.0)
        
        nav_msg.position_covariance = [3.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0]
        nav_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.pub_gps.publish(nav_msg)

    def publish_imu(self,data, current_time):
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu_link"
        # Linear Acceleration (g -> m/s^2)
        imu_msg.linear_acceleration.x = data.imu.accel_x * 9.80665
        imu_msg.linear_acceleration.y = data.imu.accel_y * 9.80665
        imu_msg.linear_acceleration.z = data.imu.accel_z * 9.80665
        # # Angular Velocity (deg/s -> rad/s)
        imu_msg.angular_velocity.x = math.radians(data.imu.gyro_x)
        imu_msg.angular_velocity.y = math.radians(data.imu.gyro_y)
        imu_msg.angular_velocity.z = math.radians(data.imu.gyro_z)
        # Orientation (Euler Deg -> Quaternion)
        # NOW USING THE IMPORTED UTILITY FUNCTION
        qx, qy, qz, qw = euler_to_quaternion(
            data.imu.roll, 
            data.imu.pitch, 
            data.imu.yaw
        )

        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        
        self.pub_imu.publish(imu_msg)

    def publish_image(self, cv_image, frame_id, publisher, time_stamp):
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            msg.header.stamp = time_stamp
            msg.header.frame_id = frame_id
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"failed to publish t he imatge :{e}")

    def cmd_vel_callback(self,msg):

        throttle = msg.linear.x 
        steering = msg.angular.z 

        self.client.send_command(steering,throttle)

    def destroy_node(self):
        self._logger().info("stopping WebRTC Bridge...")
        self.client.stop()
        super().destroy_node()
        
def main(args = None):
    rclpy.init(args=args)
    node = BridgeNode()

    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()