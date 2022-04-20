from geometry_msgs.msg import Point
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py
import yaml

def read_anchor(path):
    f = open(path, "r")
    scenario = yaml.safe_load(f)

    # Import position of ancors
    anchor1 = Point()
    anchor1.x = scenario['uwb_bases']["anchor_0"]["position"][0]
    anchor1.y = scenario['uwb_bases']["anchor_0"]["position"][1]
    anchor1.z = scenario['uwb_bases']["anchor_0"]["position"][2]


    anchor2 = Point()
    anchor2.x = scenario['uwb_bases']["anchor_1"]["position"][0]
    anchor2.y = scenario['uwb_bases']["anchor_1"]["position"][1]
    anchor2.z = scenario['uwb_bases']["anchor_1"]["position"][2]

    return anchor1, anchor2

def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def read_bag(filename):
    # Open the rosbag folder
    RESOURCES_PATH = '/home/usiusi/Desktop/ros2_ws/'
    bag_path = RESOURCES_PATH + filename
    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_values = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_values[i].name: topic_values[i].type for i in range(len(topic_values))}
    imu_array = []
    wrist_pos = []
    base_0_val = []
    base_1_val = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        # Export the information given by the IMU and create an array with them
        if topic == '/user1/imu':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            imu_array.append(msg.quaternion)
        # Export the information given by the range of ancor and create an array with them 
        elif topic == '/user1/tag/ranges':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            base_0_val.append(msg.range[0])
            base_1_val.append(msg.range[1])
        # Export the information given by the wrist_gt and create an array with them
        elif topic == '/user1/wrist_gt':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            wrist_pos.append(msg.point)
    return imu_array, wrist_pos, base_0_val, base_1_val
