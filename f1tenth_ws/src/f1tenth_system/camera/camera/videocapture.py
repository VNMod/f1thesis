import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.counter = 0
        self.output_folder = 'output_images'  # Specify the output folder here
        os.makedirs(self.output_folder, exist_ok=True)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        image_path = os.path.join(self.output_folder, f'image_{self.counter:04d}.png')
        cv2.imwrite(image_path, cv_image)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    node.create_subscription(Image, 'camera/image_raw', node.image_callback, 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
