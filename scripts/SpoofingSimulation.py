#!/usr/bin/env python3

import struct, os, csv, datetime, time
import functions.calc_smvs
import functions.spoofing_sim
import registration_separated
import functions.bag2array
from multiprocessing import Pool

import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path

def load_ref(filepath):
    df = pd.read_csv(filepath)
    timestamp = df["%time"]

    x = df['field.pose.position.x']
    y = df['field.pose.position.y']
    return np.array(timestamp), np.array(x), np.array(y)

def ros_now():
    return rospy.get_time()

def get_time():
    dt_now = datetime.datetime.now()
    name = str(dt_now.month) + str(dt_now.day) + str(dt_now.hour) + str(dt_now.minute) + str(dt_now.second)
    return name

def binary2float(data):
    float_value = struct.unpack('<f', data)[0]
    return float_value

def array2float(bin_array):
    float_array = np.apply_along_axis(binary2float, axis=1, arr = bin_array)
    return float_array

def distance_filter(x, y, z, min_dist, max_dist):
    dist = (x ** 2 + y ** 2) ** 0.5 # euclidian distance
    mask = (dist > min_dist) & (dist < max_dist)
    x_filtered = x[mask]
    y_filtered = y[mask]
    z_filtered = z[mask]
    return x_filtered, y_filtered, z_filtered

def height_filter(x, y, z, min_height):
    mask = (z > min_height)
    x_filtered = x[mask]
    y_filtered = y[mask]
    z_filtered = z[mask]
    return x_filtered, y_filtered, z_filtered

def choice_largest_score(list_angle, list_score):
    largest_index = list_score.index(max(list_score))
    largest_angle = list_angle[largest_index]
    return largest_angle

def write_odom_csv(time_stamp, odom_x, odom_y, odom_z, value, now):
    formatted_time = now.strftime("%m_%d_%H_%M_%S")
    smvs_directory = rospy.get_param('smvs_save_dir', '/home/')
    os.makedirs(smvs_directory, exist_ok=True)

    filename = smvs_directory + str(formatted_time) + '.csv'
    mode = 'w' if not os.path.exists(filename) else 'a'

    with open(filename, mode, newline='') as csvfile:
        if mode == "w":

            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(["timestamp", "x", "y", "z", "smvs"]) 
            csvwriter.writerow([time_stamp, odom_x, odom_y, odom_z, value]) # timestamp, value

        else:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([time_stamp, odom_x, odom_y, odom_z, value]) # timestamp, value

def write_vulnerablity_csv(odom_x, odom_y, odom_z, vec_x, vec_y, value, now):
    formatted_time = now.strftime("%m_%d_%H_%M_%S")
    vul_directory = rospy.get_param('vulnerablity_save_dir', '/home/')

    os.makedirs(vul_directory, exist_ok=True)
    filename = vul_directory + "vul_" + str(formatted_time) + '.csv'

    mode = 'w' if not os.path.exists(filename) else 'a'
    with open(filename, mode, newline='') as csvfile:
        if mode == "w":
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(["x", "y", "z", "vec_x", "vec_y", "smvs"]) 
            csvwriter.writerow([odom_x, odom_y, odom_z, vec_x, vec_y, value]) # timestamp, value
        
        else:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([odom_x, odom_y, odom_z, vec_x, vec_y, value]) # timestamp, value

def get_odom(now_time, ref_timestamp, ref_x, ref_y):
    time_diff = np.abs(ref_timestamp - now_time)
    index = np.argmin(time_diff)

    x = ref_x[index]
    y = ref_y[index]

    return x, y

class Node():
    def __init__(self):
        self.sub1 = rospy.Subscriber(rospy.get_param('subscribe_topic_name'), PointCloud2, self.subscriber_pointcloud) #subscribe topic
        self.sub2 = rospy.Subscriber(rospy.get_param('subscribe_odometry_name'), Odometry, self.odom_callback)

        self.X = None
        self.Y = None
        self.Z = None

        self.RUN_GICP = True
        self.is_publish = False

        self.lidar_start_time = None
        self.now = datetime.datetime.now()

    def odom_callback(self, msg):
        self.X = msg.pose.pose.position.x
        self.Y = msg.pose.pose.position.y
        self.Z = msg.pose.pose.position.z

    def get_imu_timestamp(self, msg):
        self.imu_timestamp = msg.header.stamp

    def subscriber_pointcloud(self, msg1):
        topic_length = int(rospy.get_param("lidar_topic_length", 18))
        iteration = int(len(msg1.data)/topic_length) # velodyne:22, livox:18
        bin = np.frombuffer(msg1.data, dtype=np.uint8) 
        bin_points = bin.reshape(iteration, topic_length) # velodyne:22, livox:18 

        x_bin = bin_points[:, 0:4]
        y_bin = bin_points[:, 4:8]
        z_bin = bin_points[:, 8:12]

        x = x_bin.view(dtype=np.float32)
        y = y_bin.view(dtype=np.float32)
        z = z_bin.view(dtype=np.float32)

        bag_time = msg1.header.stamp.secs + msg1.header.stamp.nsecs/1e9

        if self.lidar_start_time == None:
            self.lidar_start_time = bag_time

        time_stamp = float("{:.3f}".format(bag_time - self.lidar_start_time))

        coordinate_array = np.hstack((x, y, z))

        # set registration parameters
        num_iteration = int(rospy.get_param('number_of_iterations', 2))
        sample_rate = float(rospy.get_param('sample_rate', 0.5))
        scale_translation = float(rospy.get_param('noise_variance', 0.1))

        coordinate, score = registration_separated.execute_gicp(coordinate_array[:, :3], num_iteration, sample_rate, scale_translation)
        r , theta = functions.calc_smvs.cartesian2polar(coordinate[:, 0], coordinate[:, 1])

        # rosparam split_step (default = 5 deg) 
        # process5 calculate smvs score
        list_angle, list_score = functions.calc_smvs.count_eigen_score(theta, score, 5)

        largest_indice = np.argmax(np.array(list_score))
        vulnerable_direction = theta[largest_indice] # most vulnerable angle
        vec_x, vec_y = np.cos(np.radians(vulnerable_direction)), np.sin(np.radians(vulnerable_direction))

        # must set spoofing mode HFR or AHFR
        smvs = functions.calc_smvs.global_score_polar(np.array(list_score), "HFR") 

        write_odom_csv(time_stamp, self.X, self.Y, self.Z, smvs, self.now)
        write_vulnerablity_csv(self.X, self.Y, self.Z, vec_x, vec_y, smvs, self.now)

if __name__ == '__main__':
    rospy.set_param('use_sim_time', True)

    rospy.init_node('main_node')

    node = Node()

    while not rospy.is_shutdown():
        rospy.sleep(0.001)
