#!/usr/bin/python3

import rclpy
import sys
import yaml

from rclpy.node import Node
from geometry_msgs.msg import Quaternion, QuaternionStamped, PointStamped, Point
from uwb_msgs.msg import Ranges

from pose_uwb.wrist_position import wrist_position
from pose_uwb.read_file import read_anchor


class IMUSub(Node):
    anchor1 = Point()
    anchor2 = Point()
    
    wrist = PointStamped()
    current_imu = Quaternion()
    old_range_anchor1 = 0.0
    old_range_anchor2 = 0.0


    def __init__(self):
        super().__init__('wrist_coordinate')
        
        qos = rclpy.qos.QoSProfile(
            depth = 10,
            durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )

        map_path = self.declare_parameter("map_path", "").value
        self.anchor1, self.anchor2 = read_anchor(map_path)

        # self.get_logger().info('ANCHOR 1' + str(self.anchor1))
        # self.get_logger().info('ANCHOR 2' + str(self.anchor2))

        biometrics_path = self.declare_parameter("biometrics", "").value
        with open(biometrics_path, 'r') as f:
            self.Bill = yaml.safe_load(f)

        wrist_approx_topic = self.declare_parameter(
            "wrist_approx_topic", "wrist_approx"
        ).value

        self.publisher_ = self.create_publisher(
            PointStamped, 
            wrist_approx_topic,
            100)

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

        self.subscription = self.create_subscription(
            QuaternionStamped,
            imu_topic,
            self.listener_imu,
            qos)
        self.subscription  # prevent unused variable warning

        




    def listener_ranges(self, msg):
        range_anchor1 = msg.range[0]
        range_anchor2 = msg.range[1]

        if range_anchor1 == 0.0:
            range_anchor1 = self.old_range_anchor1
        if range_anchor2 == 0.0:
            range_anchor2 = self.old_range_anchor2

        self.wrist.header.stamp = msg.header.stamp
        self.wrist.header.frame_id = 'world'

        self.old_range_anchor1 = range_anchor1
        self.old_range_anchor2 = range_anchor2

        self.compute_coord(range_anchor1, range_anchor2)

        
    def listener_imu(self, msg):
        self.current_imu = msg.quaternion

        

    def compute_coord(self, range_anchor1, range_anchor2):
        
        self.wrist.point = wrist_position(self.anchor1, range_anchor1, self.anchor2, range_anchor2, self.current_imu, self.Bill)
        if self.wrist.point == Point():
            print('ERROR - NO INTERSECTION BETWEEN anchorS')

       
        self.publisher_.publish(self.wrist)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    wrist_coordinate = IMUSub()
    rclpy.spin(wrist_coordinate)
    wrist_coordinate.destroy_node()
    rclpy.shutdown()
