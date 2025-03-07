#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import base64
import numpy as np

class ImageToBase64Node(Node):
    def __init__(self):
        super().__init__('image_to_base64')

        # Obtener parámetros del launch
        self.declare_parameter("input_topic", "/apc/left/image_color")
        self.declare_parameter("output_topic", "/apc/left/image_base64")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.get_logger().info(f"📡 Suscrito a {self.input_topic}, publicando en {self.output_topic}")

        # Suscribirse al tópico de imagen
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )

        # Publicador de la imagen en Base64
        self.publisher = self.create_publisher(String, self.output_topic, 10)

    def image_callback(self, msg):
        try:
            # Convertir la imagen de ROS 2 a OpenCV
            np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            img_bgr = np_arr[:, :, ::-1]  # Convertir de BGR a RGB

            # Comprimir la imagen en JPEG y codificarla en Base64
            _, buffer = cv2.imencode('.jpg', img_bgr)
            base64_str = base64.b64encode(buffer).decode('utf-8')

            # Publicar la imagen en Base64
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
