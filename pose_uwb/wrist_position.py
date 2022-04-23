from geometry_msgs.msg import Point, Quaternion

import numpy as np
import math
import PyKDL
import warnings

warnings.filterwarnings('error')


def wrist_z_position(imu_coordinate: Quaternion, Bill):
    v = PyKDL.Vector(Bill["shoulder_to_wrist"], 0, 0)
    shoulder = Bill["footprint_to_neck"]
    R = PyKDL.Rotation.Quaternion(
        imu_coordinate.x, imu_coordinate.y, imu_coordinate.z, imu_coordinate.w
    )
    arm = R * v
    z = arm.z() + shoulder
    return z


def wrist_xy_position(pos_1: Point, dist_1: float, pos_2: Point, dist_2: float, z: float):
    p = Point()
    p.z = float(z)

    # Compute the radius of the circle at height of z
    delta_1 = np.abs(p.z - pos_1.z)
    try:
        r_1 = np.sqrt(dist_1**2 - delta_1**2)
    except Warning:
        print('WARNING: Range 1 = ', dist_1, ' Delta 1 = ', delta_1, ' SQRT problem = ', dist_1**2 - delta_1**2)
        if dist_1 == 0.0:
            r_1 = 5.0
        else:
            r_1 = dist_1

    delta_2 = np.abs(p.z - pos_2.z)
    try:
        r_2 = np.sqrt(dist_2**2 - delta_2**2)
    except Warning:
        print('WARNING: Range 2 = ', dist_2, ' Delta 2 = ', delta_2, ' SQRT problem = ', dist_2**2 - delta_2**2)
        if dist_2 == 0.0:
            r_2 = 5.0
        else:
            r_2 = dist_2


    # Define variable to make easier the computation of x and y
    a_1 = -2 * pos_1.x
    a_2 = -2 * pos_2.x

    b_1 = -2 * pos_1.y
    b_2 = -2 * pos_2.y

    c_1 = pos_1.x**2 + pos_1.y**2 - r_1**2
    c_2 = pos_2.x**2 + pos_2.y**2 - r_2**2

    # If the two ancors have the same x-coordinate then compute first y-coordinate
    # then compute the two possible x-coordinate and by scalar product decided between the two
    if a_1 == a_2:
        y = (c_2 - c_1) / (b_1 - b_2)
        p.y = float(y)
        d_1 = y**2 + b_1 * y + c_1
        d_2 = y**2 + b_2 * y + c_2

        # assert math.isclose(d_1, d_2)

        x1, x2 = np.roots([1, a_1, d_1])
        x1_, x2_ = np.roots([1, a_2, d_2])

        # assert math.isclose(x1, x1_) and math.isclose(x2, x2_)

        checker = (pos_2.x - pos_1.x) * (p.y - pos_1.y) - (pos_2.y - pos_1.y) * (
            x1 - pos_1.x
        )
        if checker > 0:
            p.x = float(x1)
        else:
            p.x = float(x2)

        # If the two ancors have the same y-coordinate then compute first x-coordinate
        # then compute the two possible y-coordinate and by scalar product decided between the two
    elif b_1 == b_2:
        x = (c_2 - c_1) / (a_1 - a_2)
        p.x = float(x)
        d_1 = x**2 + a_1 * x + c_1
        d_2 = x**2 + a_2 * x + c_2

        # assert math.isclose(d_1, d_2)

        y1, y2 = np.roots([1, b_1, d_1])
        y1_, y2_ = np.roots([1, b_2, d_2])

        # assert math.isclose(y1, y1_) and math.isclose(y2, y2_)

        checker = (pos_2.x - pos_1.x) * (y1 - pos_1.y) - (pos_2.y - pos_1.y) * (
            p.x - pos_1.x
        )
        if checker > 0:
            p.y = float(y1)
        else:
            p.y = float(y2)

    else:
        A = a_1 - a_2
        B = b_1 - b_2
        C = c_2 - c_1

        a = A**2 + B**2
        b = A**2 * b_1 - 2 * B * C - a_1 * A * B
        c = A**2 * c_1 + C**2 + a_1 * A * C

        y1, y2 = np.roots([a, b, c])

        d_1 = y1**2 + b_1 * y1 + c_1
        d_2 = y2**2 + b_1 * y2 + c_1

        x1 = compute_x(y1, p.z, a_1, d_1, pos_1, pos_2, dist_1, dist_2)
        x2 = compute_x(y2, p.z, a_1, d_2, pos_1, pos_2, dist_1, dist_2)

        if np.isnan(x1) and not np.isnan(x2):
            p.y = float(y2)
            p.x = float(x2)
        elif not np.isnan(x1) and np.isnan(x2):
            p.y = float(y1)
            p.x = float(x1)
        else:
            print("SOMETHING GONE WRONG - NO INTERSECTION")

    return p


# Given the postion of the ancor, their distance, the imu quaternion and the user biometry compute the coordinates of the wrist of the user
def wrist_position(pos_1: Point, dist_1: float, pos_2: Point, dist_2: float, imu_coordinate: Quaternion, Bill):
    # Check if the two ancors have an intersection
    if dist_1 + dist_2 > np.sqrt(np.sum(np.array([pos_1.x - pos_2.x, pos_1.y - pos_2.y, pos_1.z - pos_2.z]) ** 2,axis=0)):
        # Compute the z-coordinate of the wrist
        z = wrist_z_position(imu_coordinate, Bill)
        # Compute the coordintate of the wrist and return the entire point
        p = wrist_xy_position(pos_1, dist_1, pos_2, dist_2, z)
        return p

    else:
        print("ERROR, NO INTERSECTION OF THE SPHERES!")
        return Point()


def compute_x(y: float, z:float, a:float, d:float, pos_1:Point, pos_2:Point, dist_1:float, dist_2:float):
    x1, x2 = np.roots([1, a, d])
    checker_x1 = (pos_2.x - pos_1.x) * (y - pos_1.y) - (pos_2.y - pos_1.y) * (
        x1 - pos_1.x
    )
    checker_x2 = (pos_2.x - pos_1.x) * (y - pos_1.y) - (pos_2.y - pos_1.y) * (
        x2 - pos_1.x
    )

    dist_x1_pos1 = dist(pos_1, x1, y, z)
    dist_x1_pos2 = dist(pos_2, x1, y, z)
    dist_x2_pos1 = dist(pos_1, x2, y, z)
    dist_x2_pos2 = dist(pos_2, x2, y, z)

    if checker_x1 > 0 and checker_x2 > 0:
        if math.isclose(dist_x1_pos1, dist_1) and math.isclose(dist_x1_pos2, dist_2):
            x = x1
        elif math.isclose(dist_x2_pos1, dist_1) and math.isclose(dist_x2_pos2, dist_2):
            x = x2
        else:
            x = np.NaN
    elif checker_x1 > 0 and checker_x2 < 0:
        if math.isclose(dist_x1_pos1, dist_1) and math.isclose(dist_x1_pos2, dist_2):
            x = x1
        else:
            x = np.NaN
    else:
        if math.isclose(dist_x2_pos1, dist_1) and math.isclose(dist_x2_pos2, dist_2):
            x = x2
        else:
            x = np.NaN

    return x


def dist(pos: Point, x: float, y: float, z: float):
    first = (pos.x - x) ** 2
    second = (pos.y - y) ** 2
    third = (pos.z - z) ** 2
    d = np.sqrt(first + second + third)
    return d
