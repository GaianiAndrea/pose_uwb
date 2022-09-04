from geometry_msgs.msg import Point, Quaternion

import numpy as np
import math
import PyKDL

def compute_z_coords(imu_coordinate: Quaternion, Bill):
    v = PyKDL.Vector(Bill["shoulder_to_wrist"], 0, 0)
    shoulder = Bill["footprint_to_neck"]
    R = PyKDL.Rotation.Quaternion(
        imu_coordinate.x, imu_coordinate.y, imu_coordinate.z, imu_coordinate.w
    )
    arm = R * v
    z = arm.z() + shoulder
    return z


def compute_xy_coords(pos_1: Point, dist_1: float, pos_2: Point, dist_2: float, z: float):
    p = Point()
    p.z = float(z)


    # Define variable to make easier the computation of x and y
    a_1 = -2 * pos_1.x
    a_2 = -2 * pos_2.x

    b_1 = -2 * pos_1.y
    b_2 = -2 * pos_2.y

    c_1 = pos_1.x**2 + pos_1.y**2 - dist_1**2
    c_2 = pos_2.x**2 + pos_2.y**2 - dist_2**2

    # If the two ancors have the same x-coordinate then compute first y-coordinate
    # then compute the two possible x-coordinate and by scalar product decided between the two
    if a_1 == a_2:
        y = (c_2 - c_1) / (b_1 - b_2)
        p.y = float(y)
        d_1 = y**2 + b_1 * y + c_1
        d_2 = y**2 + b_2 * y + c_2

        x1, x2 = np.roots([1, a_1, d_1])

        checker = (pos_2.x - pos_1.x) * (p.y - pos_1.y) - (pos_2.y - pos_1.y) * (x1 - pos_1.x)
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

        y1, y2 = np.roots([1, b_1, d_1])

        checker = (pos_2.x - pos_1.x) * (y1 - pos_1.y) - (pos_2.y - pos_1.y) * (p.x - pos_1.x)
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

        if isinstance(y1, complex):
            y1 = y1.real + y1.imag
        if isinstance(y2, complex):
            y2 = y2.real + y2.imag

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
def compute_position(distance: float, pos_1: Point, dist_1: float, pos_2: Point, dist_2: float, imu_coordinate: Quaternion, Bill):
    
    # Compute the z-coordinate of the wrist
    z = compute_z_coords(imu_coordinate, Bill)
    if np.isnan(z):
        print('z is NaN')
        return Point(), 'z'

    # Compute the radius of the circle at height of z
    delta_1 = np.abs(z - pos_1.z)
    r_1 = np.sqrt(dist_1**2 - delta_1**2)

    delta_2 = np.abs(z - pos_2.z)
    r_2 = np.sqrt(dist_2**2 - delta_2**2)
    
    # Check if the two anchors have an intersection
    if r_1 + r_2 > distance and distance + r_1 > r_2 and distance + r_2 > r_1:
        # Compute the coordintate of the wrist and return the entire point
        p = compute_xy_coords(pos_1, r_1, pos_2, r_2, z)
        return p, 'ok'

    else:
        print("ERROR, NO INTERSECTION OF THE SPHERES!")
        return Point(), 'not_intersection'


def compute_x(y: float, z:float, a:float, d:float, pos_1:Point, pos_2:Point, dist_1:float, dist_2:float):
    x1, x2 = np.roots([1, a, d])

    if isinstance(x1, complex):
        x1 = x1.real + x1.imag
    if isinstance(x2, complex):
        x2 = x2.real + x2.imag

    checker_x1 = (pos_2.x - pos_1.x) * (y - pos_1.y) - (pos_2.y - pos_1.y) * (x1 - pos_1.x)
    checker_x2 = (pos_2.x - pos_1.x) * (y - pos_1.y) - (pos_2.y - pos_1.y) * (x2 - pos_1.x)

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
