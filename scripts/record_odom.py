#!/usr/bin/env python3

import os, csv
import numpy as np
import pandas as pd
import rospy
from nav_msgs.msg import Odometry

class Node():
    def __init__(self):
        self.sub = rospy.Subscriber(rospy.get_param('subscribe_odometry_name'), Odometry, self.odom_callback)

    def odom_callback(self, msg):
        time_sec = msg.header.stamp.secs
        time_nsec = msg.header.stamp.nsecs
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        filename = rospy.get_param('no_attack_trajectory', './benign_trajectory.csv')
        mode = 'w' if not os.path.exists(filename) else 'a'
        with open(filename, mode, newline='') as csvfile:
            if mode == "w":
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(["timestamp_sec", "timestamp_nsec", "x", "y", "z"]) 
                csvwriter.writerow([time_sec, time_nsec, x, y, z]) # timestamp, value
            
            else:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow([time_sec, time_nsec, x, y, z])# timestamp, value

if __name__ == '__main__':

    rospy.init_node('get_odom_node')

    node = Node()

    while not rospy.is_shutdown():
        rospy.sleep(0.001)