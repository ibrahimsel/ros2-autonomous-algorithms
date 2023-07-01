#!/usr/bin/python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from os.path import expanduser
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        self.declare_parameter("waypoints_file", expanduser("~") + "/pp_waypoints.csv")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("drive_topic", "/drive")

        # tune below parameters to suit your needs
        self.declare_parameter("L", 1.7)  # Lookahead distance
        self.declare_parameter("min_velocity", 4.0)  # the velocity
        self.declare_parameter("max_velocity", 8.0)
        self.declare_parameter("smoothing_factor", 0.8)
        self.declare_parameter("curvature_normalization_factor", 0.01)  

        self.smoothing_factor = (
            self.get_parameter("smoothing_factor").get_parameter_value().double_value
        )
        self.curvature_normalization_factor = (
            self.get_parameter("curvature_normalization_factor")
            .get_parameter_value()
            .double_value
        )
        self.drive_topic = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
        )
        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.waypoints_file = (
            self.get_parameter("waypoints_file").get_parameter_value().string_value
        )
        self.L = self.get_parameter("L").get_parameter_value().double_value
        self.min_velocity = (
            self.get_parameter("min_velocity").get_parameter_value().double_value
        )
        self.max_velocity = (
            self.get_parameter("max_velocity").get_parameter_value().double_value
        )

        self.pose_sub = self.create_subscription(
            Odometry, self.odom_topic, self.pose_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10
        )

        # if AEB is on, uncomment these
        # self.brake_sub = self.create_subscription(
        # Bool, '/emergency_braking', self.brake_callback, 100
        # )

        self.emergency_braking = False
        self.visualize_pub_marker = self.create_publisher(Marker, "/goal_waypoint", 10)
        self.velocity = 0
        self.current_waypoint_index = 0

        with open((self.waypoints_file), "r") as wp_file:
            self.waypoints = []
            contents = wp_file.read()
            lines = contents.splitlines()
            for i in lines:  # iterate over every logged waypoint
                splitted = i.split(",")
                x = float(splitted[0])
                y = float(splitted[1])
                self.waypoints.append([y, x])

    def brake_callback(self, data):
        self.emergency_braking = data.data

    def get_goal_waypoint(self, car_point):
        closest_waypoint_index = np.argmin(
            [math.dist(car_point, waypoint) for waypoint in self.waypoints]
        )
        i = closest_waypoint_index
        while True:
            distance = math.dist(car_point, self.waypoints[i % len(self.waypoints)])
            if distance >= self.L:
                self.current_waypoint_index = i % len(self.waypoints)
                break

            i += 1  # advance the index for the next iteration

        goal_waypoint = self.waypoints[self.current_waypoint_index]

        # visualize goal waypoint
        marker = Marker()
        marker.id = -2
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_waypoint[1]
        marker.pose.position.y = goal_waypoint[0]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.visualize_pub_marker.publish(marker)
        return goal_waypoint

    def pose_callback(self, pose_msg):
        # Getting the vehicle's current position
        car_x = pose_msg.pose.pose.position.x
        car_y = pose_msg.pose.pose.position.y

        # Determining the goal waypoint
        goal_waypoint = self.get_goal_waypoint([car_y, car_x])

        # Converting the vehicle's orientation from quaternion to Euler angles to get the current heading (yaw) of the vehicle
        quaternion = np.array(
            [
                pose_msg.pose.pose.orientation.x,
                pose_msg.pose.pose.orientation.y,
                pose_msg.pose.pose.orientation.z,
                pose_msg.pose.pose.orientation.w,
            ]
        )
        # sin(yaw) * cos(pitch)
        siny_cosp = 2.0 * (
            quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]
        )

        # cos(yaw) * cos(pitch)
        cosy_cosp = 1.0 - 2.0 * (
            quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]
        )
        heading_current = np.arctan2(siny_cosp, cosy_cosp)

        # Calculating the Euclidean distance between the vehicle and the goal waypoint
        euclidean_dist = math.dist(goal_waypoint, [car_y, car_x])

        # Calculating the lookahead angle between the vehicle and the goal waypoint
        lookahead_angle = np.arctan2(goal_waypoint[0] - car_y, goal_waypoint[1] - car_x)

        # Calculating the cross-track error (the lateral distance between the vehicle's current position and the line defined by the goal waypoint and the vehicle's heading)
        delta_y = euclidean_dist * np.sin(lookahead_angle - heading_current)

        # Calculating the steering angle to be sent to the vehicle's actuator
        steering_angle = self.calculate_steering_angle(self.L, delta_y)

        # Calculating the desired velocity
        self.velocity = self.calculate_velocity(delta_y)

        # Publishing the control command (steering angle and velocity) to the vehicle
        self.publish_steering(self.velocity, steering_angle)

    def calculate_steering_angle(self, L, y):
        return (2 * y) / (math.pow(L, 2))

    def calculate_velocity(self, delta_y):
        curvature = abs(delta_y) / self.L

        # Normalized curvature, 1 being maximum curvature
        normalized_curvature = min(curvature / self.curvature_normalization_factor, 1)

        # Interpolate between max and min velocity based on normalized_curvature
        target_velocity = (
            self.max_velocity * (1 - normalized_curvature)
            + self.min_velocity * normalized_curvature
        )

        self.velocity = (
            self.smoothing_factor * target_velocity
            + (1 - self.smoothing_factor) * self.velocity
        )

        if self.emergency_braking:
            return 0.0  # stop the car
        else:
            return self.velocity

    def publish_steering(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.get_logger().info(f"speed: {speed:.2f} m/s")
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pp = PurePursuit()
    rclpy.spin(pp)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
