#!/usr/bin/python3

import numpy as np
import rclpy
import sys
import yaml

from rclpy.node import Node
from geometry_msgs.msg import Quaternion, QuaternionStamped, PointStamped, Point, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from uwb_msgs.msg import Ranges

from pose_uwb.wrist_position import wrist_position
from pose_uwb.read_file import read_anchor


class IMUSub(Node):
    anchor1 = Point()
    anchor2 = Point()
    
    old_range_anchor1 = 0.0
    old_range_anchor2 = 0.0

    cov_position_wrist = [0.3661959 , -0.00461495, -0.00240522, 0.0, 0.0, 0.0, 
                          -0.00461495,  0.27244204,  0.00448328, 0.0, 0.0, 0.0, 
                          -0.00240522,  0.00448328,  0.00400645, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    def __init__(self):
        super().__init__('wrist_coordinate')
        
        qos = rclpy.qos.QoSProfile(
            depth = 10,
            durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.filter = self.declare_parameter("filter", False).value
        self.imu_msgs = self.declare_parameter("imu_msgs", False).value

        if self.filter:
            self.imu_msgs = True

        map_path = self.declare_parameter("map_path", "").value
        self.anchor1, self.anchor2 = read_anchor(map_path)
        self.distance = np.linalg.norm(np.array([self.anchor1.x - self.anchor2.x, self.anchor1.y - self.anchor2.y]))

        biometrics_path = self.declare_parameter("biometrics", "").value
        with open(biometrics_path, 'r') as f:
            self.Bill = yaml.safe_load(f)

        wrist_approx_topic = self.declare_parameter(
            "wrist_approx_topic", "wrist_approx"
        ).value
        
        if not self.filter:
            self.wrist = PointStamped()

            self.publisher_ = self.create_publisher(
                PointStamped, 
                wrist_approx_topic,
                qos)
        else:
            self.wrist = PoseWithCovarianceStamped()

            self.publisher_ = self.create_publisher(
                PoseWithCovarianceStamped, 
                wrist_approx_topic,
                qos)

        ranges_topic = self.declare_parameter(
            "ranges_topic", "ranges"
        ).value

        self.subscription = self.create_subscription(
            Ranges,
            ranges_topic,
            self.listener_ranges,
            qos)
        self.subscription  # prevent unused variable warning

        imu_topic = self.declare_parameter(
            "imu_topic", "imu"
        ).value


        if self.imu_msgs:
            self.current_imu = Imu()

            self.subscription = self.create_subscription(
                Imu,
                imu_topic,
                self.listener_imu,
                qos)
            
        else:
            self.current_imu = Quaternion()

            self.subscription = self.create_subscription(
                QuaternionStamped,
                imu_topic,
                self.listener_imu,
                qos)
        self.subscription  # prevent unused variable warning


    def listener_ranges(self, msg):
        range_anchor1 = msg.range[0]
        range_anchor2 = msg.range[1]

        self.wrist.header.stamp = msg.header.stamp
        self.wrist.header.frame_id = 'world'

        self.old_range_anchor1 = range_anchor1
        self.old_range_anchor2 = range_anchor2
        
        if range_anchor1 >= 0.2 and range_anchor1 <= 15.0 and range_anchor2 >= 0.2 and range_anchor2 <= 15.0:
            self.compute_coord(range_anchor1, range_anchor2)
        else:
            self.get_logger().info('ERROR - TOO NEAR OR TOO FAR')
            self.get_logger().info('Distance anchor 1 = ' + str(range_anchor1))
            self.get_logger().info('Distance anchor 2 = ' + str(range_anchor2))

    def listener_imu(self, msg):
        if self.imu_msgs:
            self.current_imu = msg
        else:
            self.current_imu = msg.quaternion

    def compute_coord(self, range_anchor1, range_anchor2):
        if not self.filter:
            if self.imu_msgs:
                self.wrist.point, checker = wrist_position(self.distance, self.anchor1, range_anchor1, self.anchor2, range_anchor2, self.current_imu.orientation, self.Bill)
            else:
                self.wrist.point, checker = wrist_position(self.distance, self.anchor1, range_anchor1, self.anchor2, range_anchor2, self.current_imu, self.Bill)
            if self.wrist.point == Point() and checker == 'not_intersection':
                self.get_logger().info('ERROR - NO INTERSECTION BETWEEN ANCHORS')
            elif self.wrist.point == Point() and checker == 'z':
                self.get_logger().info('ERROR - Z IS NAN')
            elif np.isnan(self.wrist.point.x) or np.isnan(self.wrist.point.y):
                self.get_logger().info('ERROR - X OR Y IS NAN')
                self.get_logger().info('X = ' + str(self.wrist.point.x))
                self.get_logger().info('Y = ' + str(self.wrist.point.y))
            else:
                self.publisher_.publish(self.wrist)
        else:
            self.wrist.pose.pose.position, checker = wrist_position(self.distance, self.anchor1, range_anchor1, self.anchor2, range_anchor2, self.current_imu.orientation, self.Bill)
            self.wrist.pose.covariance = self.cov_position_wrist
            # self.wrist.pose.pose.orientation = self.current_imu.orientation
            if self.wrist.pose.pose.position == Point() and checker == 'not_intersection':
                self.get_logger().info('ERROR - NO INTERSECTION BETWEEN ANCHORS')
            elif self.wrist.pose.pose.position == Point() and checker == 'z':
                self.get_logger().info('ERROR - Z IS NAN')
            elif np.isnan(self.wrist.pose.pose.position.x) or np.isnan(self.wrist.pose.pose.position.y):
                self.get_logger().info('ERROR - X OR Y IS NAN')
                self.get_logger().info('X = ' + str(self.wrist.pose.pose.position.x))
                self.get_logger().info('Y = ' + str(self.wrist.pose.pose.position.y))
            else:
                self.publisher_.publish(self.wrist)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    wrist_coordinate = IMUSub()
    rclpy.spin(wrist_coordinate)
    wrist_coordinate.destroy_node()
    rclpy.shutdown()
