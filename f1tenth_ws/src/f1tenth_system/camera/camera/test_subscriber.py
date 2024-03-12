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
        
        self.output_folder = os.path.join('dataset/', input("Enter the folder name: "))

        self.output_folder_path = os.path.abspath(self.output_folder)
        self.save_flag = False
        if not os.path.isdir(self.output_folder_path):
            os.makedirs(self.output_folder_path)  # make sure the directory exists

        self.get_logger().info(f'Output folder: {self.output_folder_path}')

        self.image_count = int(input("Enter the starting image count: "))
        self.subscriber = self.create_subscription(Image, 'camera/image_raw', self.frame_callback, 10)
        self.bridge = CvBridge()

    def frame_callback(self, data):
        self.get_logger().warning("Receiving RGB frame")
        current_frame = self.bridge.imgmsg_to_cv2(data)
        
        if self.save_flag:
            # save the frame to the local disk:
            image_path = os.path.join(self.output_folder_path, f'image_{self.image_count:04d}.jpg')
            cv2.imwrite(image_path, current_frame)
            self.get_logger().info(f'Saved image: {image_path}')
            self.image_count += 1
            self.save_flag = False
        
        # to show each frame:
        cv2.imshow("RGB image", current_frame)
        key = cv2.waitKey(1)  # Wait 1ms for a key press

        if key == ord('s'):  # Set the flag to True if the key pressed is 's'
            self.save_flag = True
        elif key == ord('q'):  # Quit if the key pressed is 'q'
            cv2.destroyAllWindows()

        # # to show each frame:
        # cv2.imshow("RGB image", current_frame)
        # cv2.waitKey(1)
        
        # # save the frame to the local disk:
        # image_path = os.path.join(self.output_folder_path, f'image_{self.image_count:04d}.jpg')
        # cv2.imwrite(image_path, current_frame)
        # self.get_logger().info(f'Saved image: {image_path}')
        # self.image_count += 1

        # # to show each frame:
        # cv2.imshow("RGB image", current_frame)
        # key = cv2.waitKey(0)  # Wait indefinitely for a key press

        # if key == ord('s'):  # Save the image if the key pressed is 's'
        #     # save the frame to the local disk:
        #     image_path = os.path.join(self.output_folder_path, f'image_{self.image_count:04d}.jpg')
        #     cv2.imwrite(image_path, current_frame)
        #     self.get_logger().info(f'Saved image: {image_path}')
        #     self.image_count += 1
        # elif key == ord('q'):  # Quit if the key pressed is 'q'
        #     cv2.destroyAllWindows()

        
                
def main(args=None):
    rclpy.init(args=args)
    node = test_subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
