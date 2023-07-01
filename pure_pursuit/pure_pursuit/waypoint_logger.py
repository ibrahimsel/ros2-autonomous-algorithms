#!/usr/bin/python3
import math
import rclpy
import atexit
from rclpy.node import Node
from nav_msgs.msg import Odometry
from os.path import expanduser


class WaypointLogger(Node):
    def __init__(self):
        super().__init__("waypoint_logger")
        self.get_logger().info("Created waypoint_logger node")

        self.declare_parameter("wp_file_path", expanduser("~") + "/pp_waypoints.csv")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("save_waypoint_threshold", 0.1)
        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.wp_file_path = (
            self.get_parameter("wp_file_path").get_parameter_value().string_value
        )
        self.save_waypoint_threshold = (
            self.get_parameter("save_waypoint_threshold")
            .get_parameter_value()
            .double_value
        )

        self.create_subscription(Odometry, self.odom_topic, self.save_waypoint, 100)

        self.wp_file = open(self.wp_file_path, "w")
        self.prev_waypoint = {"x": 0, "y": 0}

    def save_waypoint(self, data):
        dist = 0.0
        dx = data.pose.pose.position.x - self.prev_waypoint["x"]
        dy = data.pose.pose.position.y - self.prev_waypoint["y"]
        dist = math.pow(dx, 2) + math.pow(dy, 2)
        if dist > self.save_waypoint_threshold:
            self.wp_file.write(
                f"{data.pose.pose.position.x}, {data.pose.pose.position.y}\n"
            )
            self.prev_waypoint["x"] = data.pose.pose.position.x
            self.prev_waypoint["y"] = data.pose.pose.position.y

    def shutdown(self):
        self.wp_file.close()
        self.get_logger().info("Goodbye")


def main(args=None):
    rclpy.init(args=args)
    wl = WaypointLogger()
    atexit.register(
        wl.shutdown
    )  # run the shutdown method of our class before the program ends
    wl.get_logger().info("Saving waypoints...")
    rclpy.spin(wl)


if __name__ == "__main__":
    main()
