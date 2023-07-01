#!/usr/bin/python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveGapFollow(Node):
    def __init__(self):
        super().__init__('reactive_gap_follow')
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def lidar_callback(self, data):
        proc_ranges = self.preprocess_lidar(
            data.ranges, data.angle_min, data.angle_increment)
        min_distance = self.find_min_distance(proc_ranges)
        proc_ranges = self.safety_bubble(
            safety_bubble_radius=20, ranges=proc_ranges, min_distance_idx=proc_ranges.index(min_distance))
        max_gap = self.find_max_gap(proc_ranges)
        best_point = self.find_best_point(max_gap)
        angle = (data.angle_min +
                 (proc_ranges.index(best_point) * data.angle_increment))

        if abs(angle) > 0.5:
            speed = 1.0
        elif 0.5 < abs(angle) <= 1.5:
            speed = 1.5
        else:
            speed = 2.0
        self.get_logger().info('Publishing: angle: %f, speed: %f' % (angle, speed))
        
        self.publish_drive_msg(angle, speed)

    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        self.a_idx = int((math.radians(70) - angle_min) / angle_increment)
        self.b_idx = int((math.radians(-70) - angle_min) / angle_increment)

        proc_ranges = list(ranges)
        for i in range(len(proc_ranges)):
            if proc_ranges[i] > 3:
                proc_ranges[i] = 0
        return proc_ranges

    def safety_bubble(self, safety_bubble_radius, ranges, min_distance_idx):
        while safety_bubble_radius > 0:
            ranges[int(min_distance_idx - (safety_bubble_radius))] = 0
            safety_bubble_radius -= 2
        return ranges

    def find_min_distance(self, ranges):
        try:
            return min([i for i in ranges[self.b_idx:self.a_idx] if i != 0])
        except ValueError:
            return 0

    def find_max_gap(self, free_space_ranges):
        temp = []
        max_gap = []
        max_gap_len = -np.inf
        for i in range(self.b_idx, self.a_idx):
            if free_space_ranges[i] > 2:
                temp.append(free_space_ranges[i])
                if (len(max_gap) > max_gap_len):
                    max_gap = temp
                    max_gap_len = len(max_gap)
                else:
                    temp = []
        return max_gap

    def find_best_point(self, max_gap):
        try:
            return (max_gap[int(len(max_gap) / 2)])
        except IndexError:
            return 0

    def publish_drive_msg(self, angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.drive.steering_angle = angle
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveGapFollow()
    rclpy.spin(node)
    node.destroy_node()

