#!/usr/bin/python3

import math
import PyKDL
import rclpy
import yaml

from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, PoseStamped, Pose, Transform, PointStamped


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

def pose_from_frame(frame: PyKDL.Frame) -> Pose:
    msg = Pose()
    o = msg.orientation
    o.x, o.y, o.z, o.w = frame.M.GetQuaternion()
    p = msg.position
    p.x, p.y, p.z = frame.p
    return msg


class RayUWB(Node):

    wrist = PyKDL.Vector()
    
    def __init__(self):
        super().__init__('pointing_ray_uwb')

        self.biometrics_path = self.declare_parameter("biometrics", "").value

        pointing_ray_topic = self.declare_parameter(
            "pointing_ray_topic", "pointing_ray_uwb"
        ).value

        self.pub_pointing_ray_uwb = self.create_publisher(
            PoseStamped, 
            pointing_ray_topic, 
            100
        )

        qos = rclpy.qos.QoSProfile(
            depth = 10,
            durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )

        imu_topic = self.declare_parameter(
            "imu_topic", "imu"
        ).value

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
            100
        )
        self.sub

    def imu_to_ray(self, data: QuaternionStamped) -> None:
        rot = PyKDL.Rotation.Quaternion(
            data.quaternion.x,
            data.quaternion.y,
            data.quaternion.z,
            data.quaternion.w
        )
        self.pointing_model = PointingBill(self.wrist, rot, self.biometrics_path)
        
        ray_frame = self.pointing_model.ray()

        ray_pose = PoseStamped()
        ray_pose.header.stamp = data.header.stamp
        ray_pose.header.frame_id = 'world'
        ray_pose.pose = pose_from_frame(ray_frame)
        self.pub_pointing_ray_uwb.publish(ray_pose)


    def wrist_pos(self, data):
        
        self.wrist[0] = data.point.x
        self.wrist[1] = data.point.y
        self.wrist[2] = data.point.z

        
        

def main(args=None):
    rclpy.init(args=args)

    node = RayUWB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
