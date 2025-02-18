#!/usr/bin/env python3

from rosbags.rosbag1 import Reader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.serde import cdr_to_ros1, serialize_cdr
from rosbags.typesys.stores.ros1_noetic import std_msgs__msg__Header as Header
from rosbags.typesys.types import sensor_msgs__msg__PointCloud2 as Pcl2
from rosbags.typesys.types import sensor_msgs__msg__Imu as Imu
from rosbags.typesys.types import sensor_msgs__msg__PointField as Field
from rosbags.typesys.types import builtin_interfaces__msg__Time as Ts

import rosbags.typesys.types

import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import json, os
import functions.calc_smvs
import functions.spoofing_sim
import functions.bag2array

def binary_to_xyz(binary):
    x = binary[:, 0:4].view(dtype=np.float32)
    y = binary[:, 4:8].view(dtype=np.float32)
    z = binary[:, 8:12].view(dtype=np.float32)
    return x.flatten(), y.flatten(), z.flatten()

def load_reference(csv_file_name):
    df = pd.read_csv(csv_file_name)
    return df

def compare_reference(rosbag_time, dataframe_reference):
    reference_time = dataframe_reference['timestamp']
    x, y = np.array(dataframe_reference['x']), np.array(dataframe_reference['y'])

    # find corresponding timestamp
    time_diff = np.abs(reference_time - rosbag_time)
    corresponding_index = np.argmin(time_diff)

    return x[corresponding_index], y[corresponding_index]

def check_spoofing_condition(odom_x, odom_y, spoofer_x, spoofer_y, distance_threshold):
    dist_spoofer_to_robot = ((odom_x - spoofer_x) ** 2 + (odom_y - spoofer_y) ** 2) ** 0.5

    if dist_spoofer_to_robot <= distance_threshold:
        return True
    else:
        return False
    
def decide_spoofing_param(odom_x, odom_y, spoofer_x, spoofer_y):
    spoofing_angle = np.degrees(np.arctan2(spoofer_y - odom_y, spoofer_x - odom_x)) 
    return spoofing_angle

def set_timestamp(timestamp):
    sec = int(timestamp // 1000000000)        
    nanosec = int(timestamp % 1000000000)     
    return sec, nanosec

def main():
    # Create a typestore and get the string class.
    config_file = rospy.get_param('config_file', '/home/') 

    typestore = get_typestore(Stores.ROS1_NOETIC) 
    Pointcloud = typestore.types['sensor_msgs/msg/PointCloud2']
    Imu = typestore.types['sensor_msgs/msg/Imu']

    # load config file
    with open(config_file, 'r') as f:
        config = json.load(f)

    if os.path.exists(config['main']['output_file']):
        os.remove(config['main']['output_file'])

    # LiDAR params
    topic_length = config['main']['lidar_topic_length']
    start_time = None
    reference_file = "/home/rokuto/rosbag_editer/benign_brute.csv"
    spoofing_mode = config['main']['spoofing_mode'] # removal or static or dynamic
    spoofer_x = config['main']['spoofer_x']
    spoofer_y = config['main']['spoofer_y']
    distance_threshold = config['main']['distance_threshold'] # unit:m

    imu_topic = config['main']['imu_topic']
    points_topic = config['main']['lidar_topic']

    # filtering settings
    min_dist = config['filtering']['minimum_measuring_distance']
    max_dist = config['filtering']['maximum_measuring_distance']
    min_height = config['filtering']['minimum_height_threshold']

    # visualize setting
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(1, 1, 1)

    dataframe_reference = load_reference(reference_file)

    with Reader(config['main']['input_file']) as reader:
        with Writer(config['main']['output_file']) as writer:
            connections = {}

            # For each message in the input bag
            for connection, timestamp, rawdata in reader.messages():

                if connection.topic != imu_topic and connection.topic != points_topic:
                    continue

                # Create a writer connection if it does not exist
                if connection.topic not in connections:
                    print('add connection (topic={})'.format(connection.topic))
                    msgtype = Pcl2.__msgtype__ if connection.topic == points_topic else Imu.__msgtype__
                    connections[connection.topic] = writer.add_connection(connection.topic, msgtype, typestore=typestore)

                # Modify the message if it is PointCloud2
                if connection.topic == points_topic:
                    ax.cla()
                    points_msg = typestore.deserialize_ros1(rawdata, connection.msgtype)

                    now_time = timestamp/1e9

                    if start_time == None:
                        start_time = now_time

                    rosbag_time = now_time - start_time
                    odom_x, odom_y = compare_reference(rosbag_time, dataframe_reference)
                    is_spoofing = check_spoofing_condition(odom_x, odom_y, spoofer_x, spoofer_y, distance_threshold)

                    iteration = int(len(points_msg.data)/topic_length) # velodyne:22, livox:18
                    bin_points = np.frombuffer(points_msg.data, dtype=np.uint8).reshape(iteration, topic_length) # velodyne:22, livox:18 
                    x, y, z = binary_to_xyz(bin_points)

                    # distance filter
                    x_temp, y_temp, z_temp = functions.bag2array.distance_filter(x, y, z, min_dist, max_dist)

                    # ground filter
                    x_filtered, y_filtered, z_filtered = functions.bag2array.height_filter(x_temp, y_temp, z_temp, min_height)
                    coordinate_array = np.vstack((x_filtered, y_filtered, z_filtered)).T

                    if is_spoofing and spoofing_mode == "removal":
                        spoofing_angle = decide_spoofing_param(odom_x, odom_y, spoofer_x, spoofer_y)
                        x_spoofed, y_spoofed, z_spoofed = functions.spoofing_sim.spoof_main(coordinate_array, spoofing_angle, config['main']['spoofing_range'])

                    elif is_spoofing and spoofing_mode == "static":
                        spoofing_angle = decide_spoofing_param(odom_x, odom_y, spoofer_x, spoofer_y)
                        x_spoofed, y_spoofed, z_spoofed =functions.spoofing_sim.injection_main(coordinate_array, spoofing_angle, config['main']['spoofing_range'], config['main']['wall_dist'])
                        
                    else:
                        x_spoofed, y_spoofed, z_spoofed = x, y, z

                    spoofed_cloud = np.vstack((x_spoofed.astype(np.float32), y_spoofed.astype(np.float32), z_spoofed.astype(np.float32))).T

                    points = spoofed_cloud.reshape(-1).view(np.uint8)

                    MSG = Pcl2(
                    Header(seq=points_msg.header.seq, stamp=points_msg.header.stamp, frame_id='velodyne'),
                    height=1, width=int(spoofed_cloud.shape[0]),
                    fields=[
                        Field(name='x', offset=0, datatype=7, count=1),
                        Field(name='y', offset=4, datatype=7, count=1),
                        Field(name='z', offset=8, datatype=7, count=1)],
                    is_bigendian=False, point_step=12,
                    row_step=int(spoofed_cloud.shape[0]) * 12, data=points, is_dense=True)

                    rawdata = typestore.serialize_ros1(MSG, connection.msgtype)

                    ax.scatter(x_spoofed, y_spoofed, s=1)
                    ax.set_xlabel('x')
                    ax.set_ylabel('y')
                    plt.pause(0.05)

                    # Write the message as is
                    writer.write(connections[connection.topic], timestamp, rawdata)
                elif connection.topic == imu_topic:
                    writer.write(connections[connection.topic], timestamp, rawdata)
                else:
                    writer.write(connections[connection.topic], timestamp, rawdata)
    
    plt.show()

if __name__ == '__main__':
    main()
