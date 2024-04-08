#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# from std_msgs.msg import String

# Vehicle dynamic params
# MAX_VELOCITY = 1.5 # meters per second
MAX_VELOCITY = 1.0 # meters per second
MAX_STEERING_ANGLE = 0.35
# WALL FOLLOW PARAMS
DESIRED_DISTANCE_RIGHT = 0.5  # meters
DESIRED_DISTANCE_LEFT = 0.5
PID_PREDICT_DISTANCE = 1.0
MAX_INTEGRAL = 2.0
# PID CONTROL PARAMS
kp = 0.8
kd = 0.002
ki = 0.005
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0
latest_drive_velocity = 0.0
safe_brake_flag = False
# Pure pursuit params
PURE_PURSUIT_THRESHOLD = 0.15
PURE_PURSUIT_START = math.radians(-90)
PURE_PURSUIT_END = math.radians(90)
# Safe brake params
ENABLE_SAFE_BRAKE = False
VEHICLE_LENGTH = 0.55
VEHICLE_WIDTH = 0.3
SAFE_BRAKE_LIDAR_OFFSET = 0.15
MAX_DECELERATION = 7.0
ALL_POSSIBLE_DELAY = 0.05
SAFE_BRAKE_THRESHOLD = 0.03

SAFE_BRAKE_MSG = AckermannDriveStamped()
SAFE_BRAKE_MSG.header.frame_id = "laser"
SAFE_BRAKE_MSG.drive.steering_angle = 0.0
SAFE_BRAKE_MSG.drive.speed = 0.0


class WallFollow(Node):
    """Implement Wall Following on the car"""

    def __init__(self):
        """'initialize node, publishers and subscribers"""

        super().__init__('wall_follower')
        # Topics & Subs, Pubs
        lidar_scan_topic = '/scan'
        # software_drive_topic = '/nav'  # used in simulation
        software_drive_topic = 'drive' # used in real vehicle
        # vesc_drive_topic = '/drive'  # used in simulation
        vesc_drive_topic = 'ackermann_cmd' # used in real vehicle
        reset_safe_brake_topic = '/reset_safe_brake'

        # self.software_drive_pub = rospy.Publisher(software_drive_topic, AckermannDriveStamped, queue_size=10)
        # self.lidar_sub = rospy.Subscriber(lidar_scan_topic, LaserScan, self.lidar_callback)
        # self.vesc_drive_sub = rospy.Subscriber(vesc_drive_topic, AckermannDriveStamped, self.vesc_drive_callback)
        # self.safe_brake_reset = rospy.Subscriber(reset_safe_brake_topic, Bool, self.reset_safe_brake_callback)

        self.software_drive_pub = self.create_publisher(AckermannDriveStamped, software_drive_topic, 10)
        self.lidar_sub = self.create_subscription(LaserScan, lidar_scan_topic, self.lidar_callback, 10)
        self.vesc_drive_sub =  self.create_subscription(AckermannDriveStamped, vesc_drive_topic, self.vesc_drive_callback, 10)
        self.safe_brake_reset = self.create_subscription(Bool, reset_safe_brake_topic, self.reset_safe_brake_callback, 10)
        
        # prevent unused warning
        self.software_drive_pub
        self.lidar_sub
        self.vesc_drive_sub
        self.safe_brake_reset

    def get_range(self, data, angle):
        """Get the range at target angle

        Parameters:
        ---
        data: single message from topic /scan
        angle: between -135 to 135 degrees, where -90 degrees is directly to the right
        
        Returns:
        ---
        length in meters to object with angle in lidar scan field of view, 0 if nan or out of range
        """

        # data = LaserScan(data)
        idx = int((math.radians(angle) - data.angle_min) / data.angle_increment)
        if np.isinf(data.ranges[idx]) or np.isnan(data.ranges[idx]):
            return 0
        if data.ranges[idx] > data.range_max or data.ranges[idx] < data.range_min:
            return 0
        return data.ranges[idx]

    def pid_control(self, error, max_velocity):
        """Calculate the pid control output for

        Parameters:
        ---
        error: current error
        max_velocity: maximul speed

        Returns:
        ---
        drive_msg: AckermannDriveStamped()
            desired drive message of pid control
        """

        global integral
        global prev_error
        global kp
        global ki
        global kd

        integral += error
        if integral > MAX_INTEGRAL:
            integral = MAX_INTEGRAL
        elif integral < -MAX_INTEGRAL:
            integral = -MAX_INTEGRAL

        angle = (kp * error + ki * integral + kd * (error - prev_error)) / max_velocity

        if angle > MAX_STEERING_ANGLE:
            angle = MAX_STEERING_ANGLE
        elif angle < -MAX_STEERING_ANGLE:
            angle = -MAX_STEERING_ANGLE

        if math.fabs(angle) <= math.radians(10):
            velocity = max_velocity
        elif math.fabs(angle) <= math.radians(20):
            velocity = max_velocity * 0.7
        else:
            velocity = max_velocity * 0.3

        # self.get_logger().info("PID: error %4f, integral %4f, diff %4f,angle %4f." % (error, integral, error-prev_error, angle))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        prev_error = error

        return drive_msg

    def follow_right(self, scan):
        """get the distance error to right-hand wall 

        First, the distance to the wall is calculated.
        Second, predict for a specific distance assuming the speed and steering won't change
        Finally, the error is get by 'DESIRED_DISTANCE_RIGHT - d_pre'

        Parameters:
        ---
        scan: sensor_msgs.msg.LaserScan
            the lidar scan message

        Returns:
        ---
        distance error
        """
        
        angle_b = -90.0
        angle_a = -40.0
        while angle_a <= 135 and self.get_range(scan, angle_b) == 0:
            angle_b += 0.25
        while angle_b <= 135 and self.get_range(scan, angle_a) == 0:
            angle_a += 0.25
        a = self.get_range(scan, angle_a)
        b = self.get_range(scan, angle_b)
        theta = math.radians(angle_a - angle_b)
        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        d = b * math.cos(alpha)
        self.get_logger().info("Distance to right-hand wall is: %4f." % d)
        d_pre = d + PID_PREDICT_DISTANCE * math.sin(alpha)
        return DESIRED_DISTANCE_RIGHT - d_pre

    def follow_the_gap(self, scan):
        """Follow the Gap controller of the car.
        Find and follow the largest gap between obstacles.

        Parameters:
        ---
        scan: laser scan message

        Return:
        ---
        drive_msg: AckermannDriveStamped()
            desired drive message for following the gap
        """
        # Constants
        MAX_GAP_WIDTH = 0.5  # Maximum allowable gap width to consider

        # Initialize variables
        largest_gap_width = 0
        largest_gap_start = 0
        largest_gap_end = 0

        # Iterate through the laser scan data to find the largest gap
        start_index = 0
        end_index = len(scan.ranges)
        i = start_index

        while i < end_index:
            # Skip invalid or nan values
            if math.isinf(scan.ranges[i]) or math.isnan(scan.ranges[i]):
                i += 1
                continue

            # Find the start of a potential gap
            if scan.ranges[i] > MAX_GAP_WIDTH:
                gap_start_index = i

                # Find the end of the potential gap
                while i < end_index and scan.ranges[i] > MAX_GAP_WIDTH:
                    i += 1

                # Calculate the width of the potential gap
                gap_width = i - gap_start_index

                # Update largest gap if this gap is wider
                if gap_width > largest_gap_width:
                    largest_gap_width = gap_width
                    largest_gap_start = gap_start_index
                    largest_gap_end = i - 1

            i += 1

        # Calculate the angle of the midpoint of the largest gap
        gap_midpoint_index = (largest_gap_start + largest_gap_end) // 2
        angle = scan.angle_min + gap_midpoint_index * scan.angle_increment

        # Limit the steering angle to avoid extreme values
        angle = max(min(angle, MAX_STEERING_ANGLE), -MAX_STEERING_ANGLE)

        # Set velocity based on steering angle
        if abs(angle) <= math.radians(10):
            velocity = MAX_VELOCITY
        else:
            velocity = MAX_VELOCITY * (0.7 + 0.3 * (MAX_STEERING_ANGLE - abs(angle)) / (MAX_STEERING_ANGLE - math.radians(10)))

        # Create the drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity

        return drive_msg


    def safe_controller(self, scan):
        """safety controller for the car

        Parameters:
        ---
        scan: laser scan message

        Return:
        ---
        a boolean value indicating whether activate safe brake
        """

        # scan = LaserScan() # used for autocompletion
        global latest_drive_velocity
        global safe_brake_flag
        global SAFE_BRAKE_LIDAR_OFFSET
        global MAX_DECELERATION
        global ALL_POSSIBLE_DELAY
        global SAFE_BRAKE_THRESHOLD

        if safe_brake_flag:
            return True

        min_ttc = 999999
        skip_laser = 1
        inc_angle = skip_laser * scan.angle_increment

        angle = scan.angle_min - inc_angle

        for i in range(0, len(scan.ranges), skip_laser):
            angle += inc_angle
            if math.isinf(scan.ranges[i]) or math.isnan(scan.ranges[i]):
                continue
            relative_velocity = latest_drive_velocity * math.cos(angle)
            if (relative_velocity <= 0):
                continue
            ttc = (scan.ranges[i] - SAFE_BRAKE_LIDAR_OFFSET) / relative_velocity
            if ttc < min_ttc:
                min_ttc = ttc
                # self.get_logger().info("find less ttc with range idx %d : %4f, relative speed: %4f, ttc is: %4f." % (i, scan.ranges[i], relative_velocity, ttc))
        stop_time = latest_drive_velocity / MAX_DECELERATION + ALL_POSSIBLE_DELAY

        # self.get_logger().info("latest speed is %4f,   min ttc is %4f." % (latest_drive_velocity, min_ttc))
        # self.get_logger().info("stop time %4f." % (stop_time))
        
        # self.get_logger().info("ttc is %f, stop time is %f, threshold is %f." % (min_ttc, stop_time, SAFE_BRAKE_THRESHOLD))
        if min_ttc - stop_time < SAFE_BRAKE_THRESHOLD:
            safe_brake_flag = True
            return True
        return False

    def lidar_callback(self, data):
        """lidar msg callback function
        
        calculate pid control and
        guard safety
        """
        global ENABLE_SAFE_BRAKE
        
        self.get_logger().info("Doing lidar work.")
        # send error to pid_control
        # pid_drive_msg = self.pid_control(self.follow_right(data), MAX_VELOCITY)
        # self.software_drive_pub.publish(pid_drive_msg)
        
        if ENABLE_SAFE_BRAKE:
            need_safe_brake = self.safe_controller(data)
            if need_safe_brake:
                SAFE_BRAKE_MSG.header.stamp = self.get_clock().now().to_msg()
                self.software_drive_pub.publish(SAFE_BRAKE_MSG)
                self.get_logger().info("Safe brake activated.")
            else:
                pure_pursuit_msg = self.pure_pursuit(data)
                self.software_drive_pub.publish(pure_pursuit_msg)
        else:
            pure_pursuit_msg = self.pure_pursuit(data)
            self.software_drive_pub.publish(pure_pursuit_msg)

    def vesc_drive_callback(self, data):
        """vesc drive callback function

        record the latest speed of the vehicle(using the control data)

        TODO: tune the imu and change this to the feedback spped
        """

        global latest_drive_velocity
        # data = AckermannDriveStamped() # used for autocompletion
        latest_drive_velocity = data.drive.speed

    def reset_safe_brake_callback(self, data):
        """reset safe brake
        Every nonempty message will switch activation status between
        activated and deactivated
        """
       
        global safe_brake_flag
        global ENABLE_SAFE_BRAKE
        if data.data:
            safe_brake_flag = False
            if ENABLE_SAFE_BRAKE:
                ENABLE_SAFE_BRAKE = False
            else:
                ENABLE_SAFE_BRAKE = True
    
    def reset_pid_param(self):
        """reset all pid param after reseting safe brake flag"""
        global prev_error, integral

        prev_error = 0.0
        integral = 0.0


def main(args=None):
    """main function"""

    rclpy.init(args=args)
    wf = WallFollow()
    # rclpy.sleep(0.1)
    rclpy.spin(wf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
