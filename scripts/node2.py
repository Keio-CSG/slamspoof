#!/usr/bin/env python3

import struct, os, csv, datetime
import functions.calc_attackablity
import functions.spoofing_sim
import registration_separated
import functions.bag2array
from multiprocessing import Pool

import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path

def load_ref(filepath):
    df = pd.read_csv(filepath)
    timestamp = df['timestamp']
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

def write_csv(time_start, time_end, time_stamp, odom_x, odom_y, odom_z, value):
    filename = rospy.get_param('attackablity_save_dir', '/home/rokuto') + str(round(time_start, 3)) +  '_' +  str(round(time_end, 3)) + '.csv'
    mode = 'w' if not os.path.exists(filename) else 'a'
    with open(filename, mode, newline='') as csvfile:
        if mode == "w":
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(["timestamp", "x", "y", "z", "attackablity"]) 
            csvwriter.writerow([time_stamp, odom_x, odom_y, odom_z, value]) # timestamp, value
        
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([time_stamp, odom_x, odom_y, odom_z, value]) # timestamp, value

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
        
        self.pub1 = rospy.Publisher("/largest_score_angle", String, queue_size=10) # publish topic 1
        self.pub2 = rospy.Publisher("/Attackablity", String, queue_size=10) # publisht topic 2

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

        # pointcloud publish settings
        self.dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4')] 
        self.field = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

        self.pub3 = rospy.Publisher("/cloud_spoofed", PointCloud2, queue_size=10)

    def odom_callback(self, msg):
        self.X = msg.pose.pose.position.x
        self.Y = msg.pose.pose.position.y
        self.Z = msg.pose.pose.position.z

    def subscriber_pointcloud(self, msg1):
        global spoofing_start_time

        iteration = int(len(msg1.data)/18) # velodyne:22, livox:18
        bin = np.frombuffer(msg1.data, dtype=np.uint8) # 点群topicの生データをbyte列に直したもの
        bin_points = bin.reshape(iteration, 18) # velodyne:22, livox:18 点群データを各点の情報として取り出す

        x_bin = bin_points[:, 0:4]
        y_bin = bin_points[:, 4:8]
        z_bin = bin_points[:, 8:12]

        p = Pool(processes=3)
        bin_list = [x_bin, y_bin, z_bin]
        float_array = (p.map(functions.bag2array.array2float, bin_list)) 
        x = float_array[0]
        y = float_array[1]
        z = float_array[2]
        p.close()
        p.join()

        # distance filter
        min_dist =  int(rospy.get_param('min_distance', 0))
        max_dist =  int(rospy.get_param('max_distance', 1000))
        x_temp, y_temp, z_temp = functions.bag2array.distance_filter(x, y, z, min_dist, max_dist)

        # ground filter
        min_height = float(rospy.get_param('min_height', -1.0))
        x_filtered, y_filtered, z_filtered = functions.bag2array.height_filter(x_temp, y_temp, z_temp, min_height)

        coordinate_array = np.vstack((x_filtered, y_filtered, z_filtered)).T

        # set registration parameters
        num_iteration = int(rospy.get_param('number_of_iterations', 2))
        sample_rate = float(rospy.get_param('sample_rate', 0.5))
        scale_translation = float(rospy.get_param('noise_variance', 0.1))

        coordinate, score = registration_separated.execute_gicp(coordinate_array[:, :3], num_iteration, sample_rate, scale_translation) #GICPの実行. 各点score算出

        # start = time.time()
        r , theta = functions.calc_attackablity.cartesian2polar(coordinate[:, 0], coordinate[:, 1])

        # rosparam split_step (default = 5 deg) 小領域に分割し、Attackablity計算を行う準備
        list_angle, list_score = functions.calc_attackablity.count_eigen_score(theta, score, 5)

        # must set spoofing mode HFR or AHFR
        attackablity = functions.calc_attackablity.global_score_polar(np.array(list_score), "HFR") # Attackablityを計算する
        time_stamp = float("{:.3f}".format(ros_now() - start))

        # spoofed points publish
        header_seq = msg1.header.seq
        header_stamp = msg1.header.stamp

        max_range = float(rospy.get_param('spoofing_max_distance', '10.0'))
        spoofer_x = float(rospy.get_param("spoofer_location_x", 0.0)) # spooferのx座標
        spoofer_y = float(rospy.get_param("spoofer_location_y", 0.0)) # spooferのy座標

        time_stamp = float("{:.3f}".format(ros_now() - start)) #rosbagを読み込んでからの経過時間

        # spoofingできる条件を決める
        odom_x, odom_y = get_odom(time_stamp, self.ref_timestamp, self.ref_x, self.ref_y)

        if time_stamp > 1: #最初の数秒は点群が読み込まれないのでエラー回避のため

            spoofer_victim_distance = ((odom_x - spoofer_x) ** 2 + (odom_y - spoofer_y) ** 2) ** 0.5
            spoofing_angle = np.degrees(np.arctan2(odom_y - spoofer_y, odom_x - spoofer_x)) + 180 #座標系をspoofingシミュレータと揃えた
            spoofing_angular_criterion = spoofing_angle % 180

            if spoofer_victim_distance <= max_range and self.min_spoofing_angle <= abs(spoofing_angular_criterion) and abs(spoofing_angular_criterion) <= self.max_spoofing_angle:
                print("spoofing angle:{}".format(round(spoofing_angle, 2)))

                # spoofingの種類を決定する
                if self.spoofing_mode == "removal":
                    x_spoofed, y_spoofed, z_spoofed = functions.spoofing_sim.spoof_main(coordinate_array, spoofing_angle, 80)

                elif self.spoofing_mode == "injection":
                     x_spoofed, y_spoofed, z_spoofed = functions.spoofing_sim.injection_main(coordinate_array, spoofing_angle, 80, self.injection_dist)

                #x_spoofed, y_spoofed, z_spoofed = functions.spoofing_sim.spoof_main(coordinate_array, 180, 90) # spoofing test mode(消し続けるシミュレーション用)
                write_csv(spoofer_x, spoofer_y, time_stamp, self.X, self.Y, self.Z, attackablity)

            else:
                #print("no spoofing")
                #x_spoofed, y_spoofed, z_spoofed = functions.spoofing_sim.spoof_main(coordinate_array, 180, 90) # spoofing test mode(消し続けるシミュレーション用)
                x_spoofed, y_spoofed, z_spoofed = coordinate_array[:, 0], coordinate_array[:, 1], coordinate_array[:, 2]
                write_csv(spoofer_x, spoofer_y, time_stamp, self.X, self.Y, self.Z, 0)
                #write_csv(spoofer_x, spoofer_y, time_stamp, self.X, self.Y, self.Z, attackablity)
        
        else:
            #x_spoofed, y_spoofed, z_spoofed = functions.spoofing_sim.spoof_main(coordinate_array, 180, 90) # spoofing test mode(消し続けるシミュレーション用)
            x_spoofed, y_spoofed, z_spoofed = coordinate_array[:, 0], coordinate_array[:, 1], coordinate_array[:, 2]
            write_csv(spoofer_x, spoofer_y, time_stamp, self.X, self.Y, self.Z, 0)
            #write_csv(spoofer_x, spoofer_y, time_stamp, self.X, self.Y, self.Z, attackablity)

        pointcloud_data = np.zeros(int(x_spoofed.shape[0]), dtype=self.dtype)
        pointcloud_data['x'] = x_spoofed
        pointcloud_data['y'] = y_spoofed
        pointcloud_data['z'] = z_spoofed

        self.publish(pointcloud_data, header_seq, header_stamp)
    
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
        self.pub3.publish(data_pub)


if __name__ == '__main__':
    global start

    ref_trajectory_path = "/home/rokuto/cbulid_ws/src/spoofing_sim/0.0_0.0.csv"
    time_stamp, odom_x, odom_y = load_ref(ref_trajectory_path)

    rospy.init_node('test_node')

    start = ros_now()
    node = Node(time_stamp, odom_x, odom_y)

    while not rospy.is_shutdown():
        rospy.sleep(0.001)