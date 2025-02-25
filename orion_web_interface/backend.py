import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64

class WebInterface(Node):
    def __init__(self):
        super().__init__('web_interface')
        self.speech_pub = self.create_publisher(String, '/speech', 10)
        self.speech_sub = self.create_subscription(String, '/speech', self.speech_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth', self.depth_callback, 10)
        self.bridge = CvBridge()
        self.depth_image_base64 = ""

    def speech_callback(self, msg):
        self.get_logger().info(f"Recibido: {msg.data}")

    def depth_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        _, buffer = cv2.imencode('.jpg', cv_image)
        self.depth_image_base64 = base64.b64encode(buffer).decode('utf-8')

        with open("/tmp/depth_image.txt", "w") as f:
            f.write(self.depth_image_base64)

def main(args=None):
    rclpy.init(args=args)
    node = WebInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
