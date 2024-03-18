#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.speed = 0.0
        # TODO: create ROS subscribers and publishers.

        # Create Publisher for emergency braking
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Create Subscription to get odom and sensor data
        self.subscription_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = scan_msg.ranges
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        for distance, angle in zip(ranges, angles):
            range_rate = self.speed * np.cos(angle)
            if range_rate <= 0: # No need to calculate TTC
                continue
            TTC = distance / range_rate
            if TTC <= 2:
                # TODO: publish command to brake
                msg = AckermannDriveStamped()
                msg.drive.speed = 0.0
                self.get_logger().info('AEB Engaged')
                self.publisher_drive.publish(msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()