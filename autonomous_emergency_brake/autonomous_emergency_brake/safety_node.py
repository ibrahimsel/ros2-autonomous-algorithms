#!/usr/bin/python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")
        self.get_logger().info("Safety node has been started")
        self.declare_parameter("aeb_topic", "/emergency_braking")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("brake_threshold", 100.0)

        self.brake_threshold = (
            self.get_parameter("brake_threshold").get_parameter_value().double_value
        )

        self.aeb_topic = (
            self.get_parameter("aeb_topic").get_parameter_value().string_value
        )

        self.drive_topic = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
        )

        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.aeb_publisher = self.create_publisher(Bool, self.aeb_topic, 10)
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10
        )
        self.emergency_braking = False
        self.speed = 0.0

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Calculate Time To Collision
        ranges = scan_msg.ranges
        range_max = scan_msg.range_max
        range_min = scan_msg.range_min
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment

        for idx, r in enumerate(ranges):
            if np.isnan(r) or r > range_max or r < range_min:
                continue
            angle = angle_min + idx * angle_increment
            if (
                -math.pi / 6 <= angle <= math.pi / 6
            ):  # Only consider points that are within +/- 30 degrees in the front
                TTC = r / max(self.speed, 0.01)
                if TTC < self.brake_threshold:
                    self.emergency_braking = True
                    self.get_logger().info(f"brake triggered at TTC: {TTC}")
                    break

        emergency_msg = Bool()
        emergency_msg.data = self.emergency_braking
        self.aeb_publisher.publish(emergency_msg)

        if self.emergency_braking:
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0.0
            brake_msg.drive.steering_angle = 0.0
            self.drive_publisher.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
