#!/home/sameep/phd_research/sidewalkauto_ws/sidenv/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, BatteryState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np 
import math
from .robot_client import Robotclient
SERVER_OFFER_URL = "http://100.102.181.1:8080/offer" 

class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")

        self.pub_front = self.create_publisher(Image, 'camera/front/image_raw',10)
        self.pub_left = self.create_publisher(Image, 'camera/left/image_raw',10)
        self.pub_right = self.create_publisher(Image, 'camera/right/image_raw',10)


        self.pub_imu = self.create_publisher(Imu, '/imu/data',10)

        self.sub_cmd = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.cv_bridge = CvBridge()
        self.get_logger().info(f"Inializing webrtc brige to (server)")
        try:
            self.client = Robotclient(SERVER_OFFER_URL )
            self.client.start()
            self.get_logger().info("Webrtc thread started")
        except Exception as e:
            self.get_logger().error("Failed to start webrtc client: {e}")
        self.timer = self.create_timer(0.033, self.update_loop)
    def update_loop(self):
        current_time = self.get_clock().now().to_msg()
        frame_front = self.client.front_camera_frame
        if frame_front is not None:
            self.publish_image(frame_front,"front_camera_link", self.pub_front, current_time)
        frame_left = self.client.left_camera_frame
        if frame_left is not None:
            self.publish_image(frame_front,"left_camera_link", self.pub_left, current_time)
        frame_right = self.client.right_camera_frame
        if frame_right is not None:
            self.publish_image(frame_front,"right_camera_link", self.pub_right, current_time)
    def publish_image(self, cv_image, frame_id, publisher, time_stamp):
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            msg.header.stamp = time_stamp
            msg.header.frame_id = frame_id
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"failed to publish t he imatge :{e}")

    def cmd_vel_callback(self,msg):


        throttle = 0.0
        steering = 0.0

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