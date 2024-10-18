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
import open3d as o3d
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path

def load_ref(filepath):
    df = pd.read_csv(filepath)
    timestamp_sec = df['timestamp_sec']
    timestamp_nsec = df['timestamp_nsec']

    time = np.array(timestamp_sec) + (np.array(timestamp_nsec) / 1e9)
    time_zero = time[0]
    timestamp = time - time_zero

    x = df['x']
    y = df['y']
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
    #formatted_time = now.strftime("%m_%d_%H_%M_%S")
    formatted_time = "wall_" + str(rospy.get_param("injection_distance", "0"))
    filename = rospy.get_param('smvs_save_dir', '/home/') + str(formatted_time) + '.csv'
    mode = 'w' if not os.path.exists(filename) else 'a'
    with open(filename, mode, newline='') as csvfile:
        if mode == "w":
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(["timestamp", "x", "y", "z", "smvs"]) 
            csvwriter.writerow([time_stamp, odom_x, odom_y, odom_z, value]) # timestamp, value

        else:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([time_stamp, odom_x, odom_y, odom_z, value]) # timestamp, value

def write_vulnerablity_csv(time_stamp, odom_x, odom_y, odom_z, vec_x, vec_y, value, now):
    formatted_time = now.strftime("%m_%d_%H_%M_%S")
    filename = rospy.get_param('vulnerablity_save_dir', '/home/') + "vul_" + str(formatted_time) + '.csv'

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
    def __init__(self, timestamp, ref_x, ref_y):
        self.sub1 = rospy.Subscriber(rospy.get_param('subscribe_topic_name'), PointCloud2, self.subscriber_pointcloud) #subscribe topic
        self.sub2 = rospy.Subscriber(rospy.get_param('subscribe_odometry_name'), Odometry, self.odom_callback)

        self.spoofing_mode = rospy.get_param("spoofing_mode", "removal")
        self.injection_dist = float(rospy.get_param("injection_distance", 10.0))

        self.spoofing_duration = float(rospy.get_param("spoofing_duration", 10.0))
        self.min_spoofing_angle = float(rospy.get_param("min_spoofing_angle", 30))
        self.max_spoofing_angle = float(rospy.get_param("max_spoofing_angle", 150))
        self.ref_timestamp = timestamp
        self.ref_x = ref_x
        self.ref_y = ref_y

        self.X = None
        self.Y = None
        self.Z = None

        self.RUN_GICP = True

        self.now = datetime.datetime.now()

        # pointcloud publish settings
        self.dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4')] 
        self.field = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

        self.pub = rospy.Publisher("/cloud_spoofed", PointCloud2, queue_size=10)

    def odom_callback(self, msg):
        self.X = msg.pose.pose.position.x
        self.Y = msg.pose.pose.position.y
        self.Z = msg.pose.pose.position.z

    def subscriber_pointcloud(self, msg1):
        global start_time

        # process1 binary to xyz conversion
        #process1_start = time.time()
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
        #print("process1:{}sec".format(time.time() - process1_start))

        # down sampling (process2)
        #process2_start = time.time()
        pcd=o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.hstack((x, y, z)))
        downsampled_temp = pcd.voxel_down_sample(voxel_size=0.6)
        downsampled_array =np.asarray(downsampled_temp.points)
        #print("process2:{}sec".format(time.time() - process2_start))
        
        # filtering process (process3)
        #process3_start = time.time()
        # distance filter
        min_dist =  int(rospy.get_param('min_distance', 0))
        max_dist =  int(rospy.get_param('max_distance', 1000))
        x_temp, y_temp, z_temp = functions.bag2array.distance_filter(downsampled_array[:, 0], downsampled_array[:, 1], downsampled_array[:, 2], min_dist, max_dist)

        # ground filter
        min_height = float(rospy.get_param('min_height', -1.0))
        x_filtered, y_filtered, z_filtered = functions.bag2array.height_filter(x_temp, y_temp, z_temp, min_height)
        coordinate_array = np.vstack((x_filtered, y_filtered, z_filtered)).T
        #print("process3:{}sec".format(time.time() - process3_start))

        if self.RUN_GICP == True:
        # process4 point-wise score calculation
        # process4_start = time.time()
        # set registration parameters
            num_iteration = int(rospy.get_param('number_of_iterations', 2))
            sample_rate = float(rospy.get_param('sample_rate', 0.5))
            scale_translation = float(rospy.get_param('noise_variance', 0.1))

            coordinate, score = registration_separated.execute_gicp(coordinate_array[:, :3], num_iteration, sample_rate, scale_translation)
            r , theta = functions.calc_smvs.cartesian2polar(coordinate[:, 0], coordinate[:, 1])
            #print("process4:{}sec".format(time.time() - process4_start))

            # rosparam split_step (default = 5 deg) 
            # process5 calculate smvs score
            #process5_start = time.time()
            list_angle, list_score = functions.calc_smvs.count_eigen_score(theta, score, 5)

            largest_indice = np.argmax(np.array(list_score))
            vulnerable_direction = theta[largest_indice] # most vulnerable angle
            vec_x, vec_y = np.cos(np.radians(vulnerable_direction)), np.sin(np.radians(vulnerable_direction))

            # must set spoofing mode HFR or AHFR
            smvs = functions.calc_smvs.global_score_polar(np.array(list_score), "HFR") 
        
        else:
            pass

        # spoofed points publish
        header_seq = msg1.header.seq
        header_stamp = msg1.header.stamp

        max_range = float(rospy.get_param('spoofing_max_distance', '10.0'))
        spoofer_x = float(rospy.get_param("spoofer_location_x", 0.0)) 
        spoofer_y = float(rospy.get_param("spoofer_location_y", 0.0))

        time_stamp = float("{:.3f}".format(ros_now() - start_time))

        odom_x, odom_y = get_odom(time_stamp, self.ref_timestamp, self.ref_x, self.ref_y)
        #print("process5:{}sec".format(time.time() - process5_start))
        
        # process6 spoofing simulation
        #process6_start = time.time()
        if time_stamp > 1: 
            
            if self.RUN_GICP == True:
                write_vulnerablity_csv(time_stamp, self.X, self.Y, self.Z, vec_x, vec_y, smvs, self.now)

            spoofer_victim_distance = ((odom_x - spoofer_x) ** 2 + (odom_y - spoofer_y) ** 2) ** 0.5
            spoofing_angle = np.degrees(np.arctan2(odom_y - spoofer_y, odom_x - spoofer_x)) + 180
            spoofing_angular_criterion = spoofing_angle % 180

            if spoofer_victim_distance <= max_range and self.min_spoofing_angle <= abs(spoofing_angular_criterion) and abs(spoofing_angular_criterion) <= self.max_spoofing_angle:

                if self.spoofing_mode == "removal":
                    x_spoofed, y_spoofed, z_spoofed = functions.spoofing_sim.spoof_main(coordinate_array, spoofing_angle, 80)

                elif self.spoofing_mode == "injection":
                     x_spoofed, y_spoofed, z_spoofed = functions.spoofing_sim.injection_main(coordinate_array, spoofing_angle, 80, self.injection_dist)

                if self.RUN_GICP == True:
                    write_odom_csv(time_stamp, self.X, self.Y, self.Z, smvs, self.now)
                else:
                    write_odom_csv(time_stamp, self.X, self.Y, self.Z, 0, self.now)

            else:
                x_spoofed, y_spoofed, z_spoofed = coordinate_array[:, 0], coordinate_array[:, 1], coordinate_array[:, 2]
                write_odom_csv(time_stamp, self.X, self.Y, self.Z, 0, self.now)
        
        else:
            x_spoofed, y_spoofed, z_spoofed = coordinate_array[:, 0], coordinate_array[:, 1], coordinate_array[:, 2]

            if self.RUN_GICP == True:
                write_vulnerablity_csv(time_stamp, self.X, self.Y, self.Z, vec_x, vec_y, smvs, self.now)

            write_odom_csv(time_stamp, self.X, self.Y, self.Z, 0, self.now)

        #print("process6:{}sec".format(time.time() - process6_start))

        # process7 publish pointcloud
        #process7_start = time.time()
        pointcloud_data = np.zeros(int(x_spoofed.shape[0]), dtype=self.dtype)
        pointcloud_data['x'] = x_spoofed
        pointcloud_data['y'] = y_spoofed
        pointcloud_data['z'] = z_spoofed

        self.publish(pointcloud_data, header_seq, header_stamp)
        #print("process7:{}sec".format(time.time() - process7_start))
    
    def publish(self, pointcloud, seq, stamp):
        data_pub = PointCloud2()
        data_pub.header.stamp = stamp
        data_pub.header.frame_id = rospy.get_param('spoofed_frame_name', 'pc_frame') # rosparamで決められるようにする
        data_pub.height = 1
        data_pub.width = pointcloud.shape[0]
        data_pub.fields = self.field
        data_pub.is_bigendian = False
        data_pub.point_step = 12
        data_pub.row_step = 12 * pointcloud.shape[0]

        data_pub.data = pointcloud.tobytes()
        data_pub.is_dense = True
        self.pub.publish(data_pub)

if __name__ == '__main__':
    global start_time
    
    rospy.set_param('use_sim_time', False)
    ref_trajectory_path = rospy.get_param('reference_trajectory_file', '/home/hoge.csv')
    time_stamp, odom_x, odom_y = load_ref(ref_trajectory_path)

    rospy.init_node('test_node')

    start_time = ros_now()
    node = Node(time_stamp, odom_x, odom_y)

    while not rospy.is_shutdown():
        rospy.sleep(0.001)