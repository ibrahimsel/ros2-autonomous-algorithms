import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math
import time
import sys


class WallFollow(Node):
    def __init__(self) -> None:
        super().__init__("wall_follower_node")
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # parameter server
        self.declare_parameter("L", 2.0)
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.001)
        self.declare_parameter("kd", 0.005)

        self.speed_x = 0
        self.speed_y = 0
        self.steering_angle = 0
        # a is the laser beam after some theta angle (0 < theta <= 70 )
        self.a = 0
        self.b = 0  # b is the laser beam that shows us the left of the car
        self.Dt = 0  # Distance to left wall
        self.Dt_plus1 = 0  # Estimated distance to left wall after 1 seconds
        self.prev_error = 0
        self.prev_time = time.time()

    def lidar_callback(self, data):
        self.a = data.ranges[
            int((math.radians(45) - data.angle_min) / data.angle_increment)
        ]
        self.b = data.ranges[
            int((math.radians(90) - data.angle_min) / data.angle_increment)
        ]
        L = self.get_parameter("L").get_parameter_value().double_value

        theta = 45
        alpha = np.arctan(
            ((self.a * np.cos(theta)) - self.b) / (self.a * np.sin(theta))
        )
        self.Dt = self.b * np.cos(alpha)
        self.Dt_plus1 = self.Dt + L * np.sin(alpha)
        distance_to_maintain = 0.2
        error = self.calculate_error(distance_to_maintain, self.Dt_plus1)
        self.pid_control(error)

    def calculate_error(self, desired_distance, current_distance):
        return desired_distance - current_distance

    def pid_control(self, error):
        kp = self.get_parameter("kp").get_parameter_value().double_value
        ki = self.get_parameter("ki").get_parameter_value().double_value
        kd = self.get_parameter("kd").get_parameter_value().double_value
        integral = 0
        angle = 0.0

        current_time = time.time()
        delta_time = self.prev_time - current_time
        self.prev_time = current_time
        integral += self.prev_error * delta_time
        angle = -(
            (kp * error)
            + (ki * integral)
            + (kd * ((error - self.prev_error)) / delta_time)
        )
        self.prev_error = error

        if abs(angle) > 2:
            velocity = 0.5
        elif 1 < abs(angle) <= 2:
            velocity = 0.7
        else:
            velocity = 1.0

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    wf = WallFollow()
    rclpy.spin(wf)
    wf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
