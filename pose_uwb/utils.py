import PyKDL
import numpy as np

from geometry_msgs.msg import Pose


def frame_from_pose(pose: Pose) -> PyKDL.Frame:
    rot = PyKDL.Rotation.Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )    
    vect = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
    return PyKDL.Frame(V=vect, R=rot)

def pose_from_frame(frame: PyKDL.Frame) -> Pose:
    msg = Pose()
    o = msg.orientation
    o.x, o.y, o.z, o.w = frame.M.GetQuaternion()
    p = msg.position
    p.x, p.y, p.z = frame.p
    return msg

def transform_from_state(x: float, y: float):
    pos = PyKDL.Vector(x, y, 0.0)
    rot = PyKDL.Rotation()
    return PyKDL.Frame(V=pos, R=rot)

def compute_theta(pose: Pose) -> float:
    orientation_gt = PyKDL.Rotation.Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    theta, _, _ = orientation_gt.GetEulerZYX()
    theta = np.fmod(theta, 2 * np.pi)
    return theta
