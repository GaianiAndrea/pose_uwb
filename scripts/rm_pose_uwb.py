#!/usr/bin/python3

import numpy as np
import rclpy.qos
import sys

from rclpy.node import Node
from geometry_msgs.msg import (
    Quaternion,
    Point,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from uwb_msgs.msg import Ranges

from pose_uwb.compute_coords import compute_xy_coords
from pose_uwb.read_file import read_anchor


class RM_Pose(Node):
    anchor1 = Point()
    anchor2 = Point()

    old_range_anchor1 = 0.0
    old_range_anchor2 = 0.0

    rm = PoseWithCovarianceStamped()
    o = Quaternion()

    z = 0.065

    def __init__(self):
        super().__init__("rm_coord")

        z_coord = self.declare_parameter(
            "z_coord", 0.065
        ).value
        self.z = z_coord

        qos = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        )

        map_path = self.declare_parameter(
            "map_path", ""
        ).value

        self.anchor1, self.anchor2 = read_anchor(map_path)
        self.distance = np.linalg.norm(
            np.array([self.anchor1.x - self.anchor2.x, self.anchor1.y - self.anchor2.y])
        )
        # Compute the delta of the circle at height of z
        self.delta_1 = np.abs(self.z - self.anchor1.z)
        self.delta_2 = np.abs(self.z - self.anchor2.z)

        rm_coords_topic = self.declare_parameter(
            "rm_coords_topic", "pos_approx"
        ).value

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, 
            rm_coords_topic, 
            qos
        )

        ranges_topic = self.declare_parameter(
            "ranges_topic", "ranges"
        ).value

        self.subscription = self.create_subscription(
            Ranges, 
            ranges_topic, 
            self.listener_ranges, 
            qos
        )
        self.subscription  # prevent unused variable warning

        odom_topic = self.declare_parameter(
            "odom_topic", "odom"
        ).value

        self.subscription = self.create_subscription(
            Odometry, 
            odom_topic, 
            self.listener_odom, 
            qos
        )
        self.subscription  # prevent unused variable warning

    def listener_ranges(self, msg: Ranges):
        # Get the distances from anchor 1 and 2
        range_anchor1 = msg.range[0]
        range_anchor2 = msg.range[1]

        r_1 = np.sqrt(range_anchor1**2 - self.delta_1**2)
        r_2 = np.sqrt(range_anchor2**2 - self.delta_2**2)

        # Define the header for the robomaster
        self.rm.header.stamp = msg.header.stamp
        self.rm.header.frame_id = "world"

        # Save the distances
        self.old_range_anchor1 = range_anchor1
        self.old_range_anchor2 = range_anchor2

        # Check if the robomaster is too far or too near
        if (range_anchor1 >= 0.2 and range_anchor1 <= 15.0
            and range_anchor2 >= 0.2 and range_anchor2 <= 15.0
        ):
            # Compute the position of the robomaster
            if r_1 + r_2 > self.distance and self.distance + r_1 > r_2 and self.distance + r_2 > r_1:
                self.compute_coord(r_1, r_2)
            else:
                self.get_logger().info("ERROR, NO INTERSECTION OF THE SPHERES!")
        else:
            self.get_logger().info("ERROR - TOO NEAR OR TOO FAR")
            self.get_logger().info("Distance anchor 1 = " + str(range_anchor1))
            self.get_logger().info("Distance anchor 2 = " + str(range_anchor2))

    def compute_coord(self, r_1: float, r_2: float):
        self.rm.pose.pose.position = compute_xy_coords(
            self.anchor1,
            r_1,
            self.anchor2,
            r_2,
            self.z,
        )
        self.rm.pose.pose.orientation = self.o

        self.rm.pose.covariance[0] = 0.02538668
        self.rm.pose.covariance[7] = 0.03299221

        if np.isnan(self.rm.pose.pose.position.x) or np.isnan(self.rm.pose.pose.position.y):
            self.get_logger().info("ERROR - X OR Y IS NAN")
            self.get_logger().info("X = " + str(self.rm.pose.pose.position.x))
            self.get_logger().info("Y = " + str(self.rm.pose.pose.position.y))
        else:
            self.publisher_.publish(self.rm)

    def listener_odom(self, data: Odometry):
        self.o = data.pose.pose.orientation


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    rm_coordinate = RM_Pose()
    rclpy.spin(rm_coordinate)
    rm_coordinate.destroy_node()
    rclpy.shutdown()
