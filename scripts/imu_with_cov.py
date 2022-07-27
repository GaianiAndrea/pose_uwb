#!/usr/bin/python3

import rclpy
import sys

from rclpy.node import Node
from geometry_msgs.msg import Quaternion, QuaternionStamped
from sensor_msgs.msg import Imu


class IMUPub(Node):

    cov_orientation = [ 0.04016028, -0.03026918,  0.02709264,
                        -0.03026918,  0.03353304, -0.02243736,
                        0.02709264, -0.02243736,  0.04986208]
    cov_ang = [ 1.78014154e-05, 2.32756281e-05, 1.47019487e-05,
                2.32756281e-05, 4.25140569e-05, 2.66015282e-05,
                1.47019487e-05, 2.66015282e-05, 2.09566015e-05]
    cov_acc = [ 3.95730344e-03, -5.49310036e-04, 5.63636249e-06, 
                -5.49310036e-04,  3.84916421e-03,  5.62849067e-04, 
                5.63636249e-06,  5.62849067e-04,  6.47906912e-03]


    def __init__(self):
        super().__init__('imu_with_covariance')
        
        qos = rclpy.qos.QoSProfile(
            depth = 10,
            durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.filter = self.declare_parameter("filter", False).value
        self.imu_msgs = self.declare_parameter("imu_msgs", False).value

        if self.filter:
            self.imu_msgs = True


        imu_topic = self.declare_parameter(
            "imu_topic", "imu"
        ).value

        new_imu_topic = self.declare_parameter(
                "new_imu_topic", 'new_imu'
            ).value

        if self.imu_msgs:
            self.current_imu = Imu()

            self.subscription = self.create_subscription(
                Imu,
                imu_topic,
                self.listener_imu,
                qos)

            self.new_imu = Imu()
            self.publisher_ = self.create_publisher(
                Imu, 
                new_imu_topic,
                qos)
        else:
            self.current_imu = Quaternion()

            self.subscription = self.create_subscription(
                QuaternionStamped,
                imu_topic,
                self.listener_imu,
                qos)
        self.subscription  # prevent unused variable warning

    def listener_imu(self, msg):
        if self.imu_msgs:
            self.current_imu = msg

            self.new_imu = self.current_imu

            self.new_imu.angular_velocity_covariance = self.cov_ang
            self.new_imu.linear_acceleration_covariance = self.cov_acc
            self.new_imu.orientation_covariance = self.cov_orientation
            self.publisher_.publish(self.new_imu)
        else:
            self.current_imu = msg.quaternion


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    wrist_coordinate = IMUPub()
    rclpy.spin(wrist_coordinate)
    wrist_coordinate.destroy_node()
    rclpy.shutdown()
