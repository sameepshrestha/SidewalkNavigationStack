import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
# Import the DA3 API provided in your text
from depth_anything_3.api import DepthAnything3

class DepthAnything3Node(Node):
    def __init__(self):
        super().__init__('da3_node')
        
        # 1. Initialize the Model
        # CRITICAL FOR ROBOTS: Use 'DA3-Small' or 'DA3-Base' if you are on a Jetson/Laptop.
        # 'DA3-Giant' will likely crash a mobile robot's computer.
        self.get_logger().info("Loading Depth Anything 3...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Note: For navigation, you probably want the METRIC model 
        # so you know exactly how many meters away obstacles are.
        self.model = DepthAnything3.from_pretrained("depth-anything/DA3Metric-Large")
        self.model = self.model.to(self.device)
        
        # 2. Setup ROS Communication
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/da3/depth', 10)
        self.cv_bridge = CvBridge()

    def listener_callback(self, msg):
        # A. Convert ROS Image to OpenCV/Numpy
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # B. Run Inference (The code from their API)
        # Note: In a real loop, check if prediction needs a list or single image
        prediction = self.model.inference([cv_image]) 
        
        # C. Get the depth map (float32 array)
        depth_map = prediction.depth[0] # Assuming batch size 1
        
        # D. Publish back to ROS
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")
        depth_msg.header = msg.header
        self.publisher_.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthAnything3Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()