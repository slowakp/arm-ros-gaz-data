#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MotionSequence(Node):
    def __init__(self):
        super().__init__('motion_sequence')
        self.pub = self.create_publisher(Twist, '/cmd/cmd_vel', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        t = time.time() - self.start_time
        msg = Twist()

        if t < 10.0:
            # do przodu + w lewo
            msg.linear.x = 0.5
            msg.linear.y = 0.3
            msg.angular.z = 0.0
        else:
            # po okręgu w prawo
            msg.linear.x = 0.4
            msg.linear.y = 0.0
            msg.angular.z = -0.6

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MotionSequence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
