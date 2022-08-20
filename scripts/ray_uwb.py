#!/usr/bin/python3

import math
import PyKDL
import rclpy.qos
import yaml

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped, QuaternionStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from pose_uwb.utils import pose_from_frame
from std_msgs.msg import Bool, Empty


class PointingBill:
    def __init__(self, wrist, rot, biometrics_path):
        # Import the biometrics of the user
        with open(biometrics_path, 'r') as f:
            self.Bill = yaml.safe_load(f)

        self.shoulder_position = (wrist - 
                                  rot * PyKDL.Vector(self.Bill['shoulder_to_wrist'], 0, 0))
        self.shoulder_position[2] = self.Bill['footprint_to_neck']

        self.eye_position = PyKDL.Vector(
            self.shoulder_position.x(),
            self.shoulder_position.y(),
            self.Bill['footprint_to_neck'] + self.Bill['neck_to_eyes']
        )

        self.finger_position = self.shoulder_position + rot * PyKDL.Vector(self.Bill['shoulder_to_wrist'] + self.Bill['wrist_to_finger'], 0, 0)

    def ray(self) -> PyKDL.Frame:
        # The ray is aligned to its x-axis
        ray_axis = self.finger_position - self.eye_position
        ray_axis = ray_axis / ray_axis.Norm()
        # rot is the rotation that align the x-axis with the ray
        rot_angle = math.acos(PyKDL.dot(PyKDL.Vector(1, 0, 0), ray_axis))
        rot_axis = PyKDL.Vector(1, 0, 0) * ray_axis
        return PyKDL.Frame(R=PyKDL.Rotation.Rot(rot_axis, rot_angle), V=self.eye_position)


class RayUWB(Node):

    wrist = PyKDL.Vector()
    rot = PyKDL.Rotation.Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    cov = []
    
    def __init__(self):
        super().__init__('pointing_ray_uwb')
        
        qos = rclpy.qos.QoSProfile(
            depth = 10,
            durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.biometrics_path = self.declare_parameter("biometrics", "").value
        self.filter = self.declare_parameter("filter", False).value
        self.imu_msgs = self.declare_parameter("imu_msgs", False).value

        if self.filter:
            self.imu_msgs = True

            # Subscriber to odom
            odom_topic = self.declare_parameter(
                "odom_topic", "filtered"
            ).value
        
            self.sub = self.create_subscription(
                Odometry,
                odom_topic,
                self.odom_to_ray,
                qos
            )
            self.sub

            # self.create_subscription(
            #     Bool, 
            #     'button', 
            #     self.has_received_button, 
            #     qos)

            self.create_subscription(
                Empty, 
                'reset_yaw', 
                self.empty_sub, 
                qos)

            set_pose = self.declare_parameter(
            "set_pose", "set_pose"
            ).value

            self.set_pose_topic = self.create_publisher(
                PoseWithCovarianceStamped, 
                set_pose, 
                rclpy.qos.QoSProfile(
                    depth = 10,
                    durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE,
                    reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                )
            )

        else:
            imu_topic = self.declare_parameter(
                "imu_topic", "imu"
            ).value
            if self.imu_msgs:
                self.sub= self.create_subscription(
                    Imu,
                    imu_topic,
                    self.imu_to_ray,
                    qos
                )
            else:
                self.sub= self.create_subscription(
                    QuaternionStamped,
                    imu_topic,
                    self.imu_to_ray,
                    qos
                )

            wrist_approx_topic = self.declare_parameter(
                "wrist_approx_topic", "wrist_approx"
            ).value
        
            self.sub = self.create_subscription(
                PointStamped,
                wrist_approx_topic,
                self.wrist_pos,
                qos
            )
            self.sub


        pointing_ray_topic = self.declare_parameter(
            "pointing_ray_topic", "pointing_ray_uwb"
        ).value

        self.pub_pointing_ray_uwb = self.create_publisher(
            PoseStamped, 
            pointing_ray_topic, 
            qos
        )

    def imu_to_ray(self, data) -> None:
        if self.imu_msgs:
            self.rot = PyKDL.Rotation.Quaternion(
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w
            )
        else:
            self.rot = PyKDL.Rotation.Quaternion(
                data.quaternion.x,
                data.quaternion.y,
                data.quaternion.z,
                data.quaternion.w
            )


    def wrist_pos(self, data: PointStamped):
        self.wrist[0] = data.point.x
        self.wrist[1] = data.point.y
        self.wrist[2] = data.point.z

        tmp = data.header.stamp
        self.ray_publisher(tmp)

    def odom_to_ray(self, data: Odometry):
        pose = data.pose.pose
        
        self.wrist[0] = pose.position.x
        self.wrist[1] = pose.position.y
        self.wrist[2] = pose.position.z

        self.rot = PyKDL.Rotation.Quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
        )

        self.cov = data.pose.covariance

        tmp = data.header.stamp
        self.ray_publisher(tmp)

    def ray_publisher(self, tmp):
        self.pointing_model = PointingBill(self.wrist, self.rot, self.biometrics_path)
        
        ray_frame = self.pointing_model.ray()

        ray_pose = PoseStamped()
        ray_pose.header.stamp = tmp
        ray_pose.header.frame_id = 'world'
        ray_pose.pose = pose_from_frame(ray_frame)
        self.pub_pointing_ray_uwb.publish(ray_pose)

    # def has_received_button(self, msg:Bool):
    #     if msg:
    #         new_pose = PoseWithCovarianceStamped()
    #         new_pose.header.frame_id = 'world'
    #         new_pose.pose.pose.position.x = self.wrist.x()
    #         new_pose.pose.pose.position.y = self.wrist.y()
    #         new_pose.pose.pose.position.z = self.wrist.z()

    #         r, p, y  = self.rot.GetRPY()
    #         y = 0
    #         new_rot = PyKDL.Rotation.RPY(r,p,y)

    #         orient = new_rot.GetQuaternion()
    #         new_pose.pose.pose.orientation.x = orient[0]
    #         new_pose.pose.pose.orientation.y = orient[1]
    #         new_pose.pose.pose.orientation.z = orient[2]
    #         new_pose.pose.pose.orientation.w = orient[3]
    #         self.set_pose_topic.publish(new_pose)   

    def empty_sub(self, msg:Empty):
        if msg:
            self.get_logger().info("resetting yaws")
            new_pose = PoseWithCovarianceStamped()
            new_pose.header.frame_id = 'world'
            new_pose.pose.pose.position.x = self.wrist.x()
            new_pose.pose.pose.position.y = self.wrist.y()
            new_pose.pose.pose.position.z = self.wrist.z()

            r, p, y  = self.rot.GetRPY()
            y = 0
            new_rot = PyKDL.Rotation.RPY(r,p,y)

            orient = new_rot.GetQuaternion()
            new_pose.pose.pose.orientation.x = orient[0]
            new_pose.pose.pose.orientation.y = orient[1]
            new_pose.pose.pose.orientation.z = orient[2]
            new_pose.pose.pose.orientation.w = orient[3]

            new_pose.pose.covariance = self.cov
            new_pose.pose.covariance[-1] = 0.01

            self.get_logger().info(str(self.cov[-1]))

            self.set_pose_topic.publish(new_pose) 

def main(args=None):
    rclpy.init(args=args)

    node = RayUWB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
