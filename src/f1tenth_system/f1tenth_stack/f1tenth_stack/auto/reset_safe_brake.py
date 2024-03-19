#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time

reset_safe_brake_topic = '/reset_safe_brake'

class ResetSignalPublisher(Node):

    def __init__(self):
        super().__init__('reset_safe_brake_talker')
        self.pub = self.create_publisher(Bool, reset_safe_brake_topic, 10)
        self.cnt = 0

    def talk(self):
        msg = Bool()
        msg.data = True
        while rclpy.ok():
            self.pub.publish(msg)
            self.cnt += 1
            if self.cnt > 10:
                brake
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    resetter = ResetSignalPublisher()

    #rclpy.spin_once(resetter)

    resetter.talk()

    resetter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
