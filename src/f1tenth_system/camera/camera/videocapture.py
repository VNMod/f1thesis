import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import pyrealsense2 as rs
from cv_bridge import CvBridge
import numpy as np

# This file is for publishing a video stream.

# what do I need to be able to do?
# 1. capture video in real time
# 2. be able to store the video as images as I capture them
# 3. store them locally
# 4. create two comprehensive datasets: training & validation, and test

# 5. need a separate node for training a neural network with these collected images
# after it's successfully trained, I download the weights of the model
# 6. then, I need to stream video in real time again, pass the images into the model
# 7. based on the model's classification, I need a node subscribed to this video footage to compute 
# the safety score for the images
# 8. I need to store the safety score, and assign a threshold. whenever the threshold is exceeded,
# the model alerts the vehicle.

# go to the main directory: colcon build --packages-select youtube_robot --symlink-install
# when I make a change in the python file, I wouldn't have to recompile.
# don't forget to also source install file: source install/setup.bash
# then I would able to: ros2 run camera videocapture 

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # topic here is: camera/image_raw
        # qos or queue list is 10
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # runs code every 2 seconds, I could try (0.05 for smoother frames like a video)
        timer_period = 0.5
        self.bridge = CvBridge() # convert from OpenCV format to Image message type
        
        try:
            #print('this works')
            self.pipe = rs.pipeline()
            self.config = rs.config()
            # streaming 640 x 480, (no now we're streaming 320, 320)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipe.start(self.config)
            self.timer = self.create_timer(timer_period, self.timer_callback)
            
        except Exception as e:
            print(e)
            self.get_logger().error("INTEL RealSense IS NOT CONNECTED.")
        
    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        
        # ros_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        self.publisher.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))
        self.get_logger().info("Publishing the RGB image frame.")

    # previous function for capturing images:
    # def capture_image(self):
    #     pipeline = rs.pipeline()
    #     config = rs.config()
    #     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    #     pipeline.start(config)
    #     try:
    #         while True:
    #             frames = pipeline.wait_for_frames()
    #             color_frame = frames.get_color_frame()
    #             if color_frame:
    #                 color_image = np.asanyarray(color_frame.get_data())
    #                 ros_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
    #                 self.publisher.publish(ros_image)
    #     finally:
    #         pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    # node.capture_image()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
