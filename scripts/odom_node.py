#!/usr/bin/env python3

import os, datetime
import rospy
from nav_msgs.msg import Odometry

def save_tum(timestamp, x, y, z, qx, qy, qz, qw, now):
    formatted_time = now.strftime("%m_%d_%H_%M_%S")
    tum_directory = rospy.get_param('trajectory_save_dir', '/home/')
    os.makedirs(tum_directory, exist_ok=True)

    filename = tum_directory + 'tum_' + str(formatted_time) + ".txt"
    mode = 'w' if not os.path.exists(filename) else 'a'
    with open(filename, mode, newline='') as txtfile:
        line = f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n"
        txtfile.write(line)

class Node():
    def __init__(self):
        self.sub1 = rospy.Subscriber(rospy.get_param('subscribe_odometry_name'), Odometry, self.subscriber_odom) #subscribe topic
        self.now = datetime.datetime.now()
        
    def subscriber_odom(self, msg):
        sec_timestamp = msg.header.stamp.secs
        n_timestamp = msg.header.stamp.nsecs
        timestamp = str(sec_timestamp) + "." + str(n_timestamp) 

        x = str(round(float(msg.pose.pose.position.x), 3)) 
        y = str(round(float(msg.pose.pose.position.y), 3))
        z = str(round(float(msg.pose.pose.position.z), 3))
        qx = str(round(float(msg.pose.pose.orientation.x), 3))
        qy = str(round(float(msg.pose.pose.orientation.y), 3))
        qz = str(round(float(msg.pose.pose.orientation.z), 3))
        qw = str(round(float(msg.pose.pose.orientation.w), 3))

        save_tum(timestamp, x, y, z, qx, qy, qz, qw, self.now)

if __name__ == '__main__':
    rospy.init_node('odom_node')

    rospy.set_param('use_sim_time', True)
    node = Node()

    while not rospy.is_shutdown():
        rospy.sleep(0.001)