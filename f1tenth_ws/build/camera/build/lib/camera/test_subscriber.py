import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import pyrealsense2 as rs
from cv_bridge import CvBridge
import numpy as np
import os

class test_subscriber(Node):
    def __init__(self):
        super().__init__("test_subscriber")
        self.subscriber = self.create_subscription(Image, 'camera/image_raw', self.frame_callback, 10)
        self.bridge = CvBridge()
        self.image_count = 0
        self.output_folder = 'dataset'
        self.output_folder_path = os.path.abspath(self.output_folder)
        if not os.path.isdir(self.output_folder_path):
            os.makedirs(self.output_folder_path)  # make sure the directory exists

        self.get_logger().info(f'Output folder: {self.output_folder_path}')

    def frame_callback(self, data):
        self.get_logger().warning("Receiving RGB frame")
        current_frame = self.bridge.imgmsg_to_cv2(data)
        
        # to show each frame:
        cv2.imshow("RGB image", current_frame)
        cv2.waitKey(1)
        
        # save the frame to the local disk:
        image_path = os.path.join(self.output_folder_path, f'image_{self.image_count:04d}.jpg')
        cv2.imwrite(image_path, current_frame)
        self.get_logger().info(f'Saved image: {image_path}')
        self.image_count += 1
        
        
def main(args=None):
    rclpy.init(args=args)
    node = test_subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()