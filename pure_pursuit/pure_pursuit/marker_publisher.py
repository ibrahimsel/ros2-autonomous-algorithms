#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")

        # load waypoints_file from ROS parameter
        self.declare_parameter(
            "wp_file_path", os.path.expanduser("~") + "/pp_waypoints.csv"
        )
        wp_file_path = (
            self.get_parameter("wp_file_path").get_parameter_value().string_value
        )
        wp_file = open(wp_file_path, "r")

        self.waypoints = []
        contents = wp_file.read()
        lines = contents.splitlines()
        for i in lines:  # iterate over every logged waypoint
            splitted = i.split(",")
            x = float(splitted[0])
            y = float(splitted[1])
            self.waypoints.append([y, x])
        wp_file.close()

        self.visualize_pub_markerarray = self.create_publisher(
            MarkerArray, "/pure_pursuit_waypoints", 10
        )

    def publish_markers(self):
        while rclpy.ok():
            self.marker_array = MarkerArray()
            ct = 0  # counter for marker ids
            for i in self.waypoints:
                marker = Marker()
                marker.id = ct
                ct += 1
                marker.header.frame_id = "map"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = i[1]
                marker.pose.position.y = i[0]
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                self.marker_array.markers.append(marker)
            self.visualize_pub_markerarray.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    mp = MarkerPublisher()
    mp.publish_markers()
    mp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
