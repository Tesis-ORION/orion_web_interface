#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import base64
import numpy as np
from cv_bridge import CvBridge


class ImageToBase64Node(Node):
    def __init__(self):
        super().__init__('image_to_base64')
        self.bridge = CvBridge()

        # Parámetros
        self.declare_parameter("input_topic", "/image_raw")
        self.declare_parameter("output_topic", "/image_base64")
        self.declare_parameter("format_type", "rgb")  # Puede ser "rgb" o "depth"

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.format_type = self.get_parameter("format_type").value.lower()

        self.get_logger().info(f"📡 Subscribed to {self.input_topic}, publishing to {self.output_topic}, format: {self.format_type}")

        # Subscriptor y publicador
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(String, self.output_topic, 10)

    def image_callback(self, msg):
        try:
            if self.format_type == "depth":
                # Procesar imagen de profundidad (32FC1)
                depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_img = np.nan_to_num(depth_img, nan=0.0)  # Sustituir NaNs

                # Normalizar y convertir a 8-bit
                depth_normalized = np.clip(depth_img / 4.0 * 255.0, 0, 255).astype(np.uint8)
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                img_to_encode = depth_colored

            else:
                # Procesar imagen RGB (8UC3)
                img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                img_to_encode = img_bgr

            # Codificar imagen en JPEG y luego a Base64
            _, buffer = cv2.imencode('.jpg', img_to_encode)
            base64_str = base64.b64encode(buffer).decode('utf-8')

            # Publicar
            msg_out = String()
            msg_out.data = base64_str
            self.publisher.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"❌ Error procesando imagen: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageToBase64Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
