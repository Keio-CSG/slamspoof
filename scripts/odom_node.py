#!/usr/bin/env python3

import struct, os, csv, datetime
import functions.calc_smvs
import functions.spoofing_sim
import registration_separated
import functions.bag2array
from multiprocessing import Pool

import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Header, String
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path
import tf.transformations as tf

def save_tum(timestamp, x, y, z, qx, qy, qz, qw, now):
    formatted_time = now.strftime("%m_%d_%H_%M_%S")
    filename = rospy.get_param('trajectory_save_dir', '/home/') + 'tum_' + str(formatted_time) + ".txt"
    mode = 'w' if not os.path.exists(filename) else 'a'
    with open(filename, mode, newline='') as txtfile:
        txtfile.writelines([timestamp, x, y, z, qx, qy, qz, qw])

class Node():
    def __init__(self):
        self.sub1 = rospy.Subscriber(rospy.get_param('subscribe_odometry_name'), Odometry, self.subscriber_odom) #subscribe topic
        self.now = datetime.datetime.now()
        
    def subscriber_odom(self, msg):
        sec_timestamp = msg.header.stamp.secs
        n_timestamp = msg.header.stamp.nsecs
        timestamp = str(sec_timestamp) + "." + str(n_timestamp) + " "

        x = str(msg.pose.pose.position.x) + " "
        y = str(msg.pose.pose.position.y) + " "
        z = str(msg.pose.pose.position.z) + " "
        qx = str(msg.pose.pose.orientation.x) + " "
        qy = str(msg.pose.pose.orientation.y) + " "
        qz = str(msg.pose.pose.orientation.z) + " "
        qw = str(msg.pose.pose.orientation.w) + "\n"

        save_tum(timestamp, x, y, z, qx, qy, qz, qw, self.now)


if __name__ == '__main__':
    rospy.init_node('odom_node')

    node = Node()

    while not rospy.is_shutdown():
        rospy.sleep(0.001)