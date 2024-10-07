#!/usr/bin/env python3

import os, re, random
import rospy
import pandas as pd
import numpy as np
import matplotlib 
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
import matplotlib.pyplot as plt
import calc_localizability

def sort_files(dir_name):
    list_dir = os.listdir(dir_name)
    return sorted(list_dir, key=lambda s: int(re.search(r'\d+', s).group()))

def csv_2_array(file_name):
    df = pd.read_csv(file_name)
    pointcloud = df[["x", "y", "z"]]
    eigenvalue = df[["eigen_value"]]

    array_points, array_values = np.array(pointcloud), np.array(eigenvalue)

    return array_points, array_values

def calc_angle(xyz):
    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]
    angle = np.degrees(np.arctan2(y, x)) + 180
    return angle

def count_eigen_score(angle_array, eigen_array, step):
    list_angle, list_score = [], []
    for i in range(int(360 / step)):
        mask = ((angle_array >= step * i) & (angle_array < step * (i+1)))
        score_table = np.sum(eigen_array[mask])
        list_score.append(score_table)
        list_angle.append(step * (i + 0.5))
    
    return list_angle, list_score

def cartesian2polar(x, y):
    r = (x ** 2 + y ** 2) ** 0.5
    theta = np.degrees(np.arctan2(y, x)) + 180
    return r, theta

def calc_ylim(file_list, dir_name, num_sample=30):
    list_maximum_score = []
    sampled_list = random.sample(file_list, num_sample)
    for sample in sampled_list:
        sampled_path = dir_name + "/" + sample
        array_points, array_values = csv_2_array(sampled_path)
        list_angle, list_score = count_eigen_score(calc_angle(array_points), array_values, 10)
        maximum_score = max(list_score)
        list_maximum_score.append(maximum_score)
    maximum_average = sum(list_maximum_score) / num_sample
    return maximum_average * 1.5

def downsample_visualize(x, y, z, eigenvalue, rate=1.0):

    num_sample = int(x.shape[0] * rate)

    indices_1 = np.random.choice(x.shape[0], num_sample, replace=False)
    x_downsampled = x[indices_1]
    y_downsampled = y[indices_1]
    z_downsampled = z[indices_1]
    eigenvalue_downsampled = eigenvalue[indices_1]

    return x_downsampled, y_downsampled, z_downsampled, eigenvalue_downsampled

#def output_heatmap(x, y, max_distance, score_array, num_x, num_y):

dir_name = rospy.get_param('directory_name', '/home')
max_distance = float(rospy.get_param('max_distance', 10))
z_min, z_max = float(rospy.get_param('z_min', -1)), float(rospy.get_param('z_max', 1))
num_x_divide, num_y_divide = int(rospy.get_param('x_divide', 10)), int(rospy.get_param('y_divide', 10))
spoofing_mode = rospy.get_param('spoofing_mode', 'HFR')
FILES = sort_files(dir_name)
list_attackablity = []

list_flatten_x = [] #平滑化した横軸
list_flatten = [] #平滑化した縦軸
ymax = calc_ylim(FILES, dir_name)

# set gigure
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 6))
plt.subplots_adjust(wspace=0.5)
counter = 0

for file in FILES:
    ax1.clear()
    ax2.clear()
    ax3.clear()
    path = dir_name + "/" + str(file)
    array_points, array_values = csv_2_array(path)

    # preprocess to visualize
    x, y, z, eigen_value = downsample_visualize(array_points[:, 0], array_points[:, 1], array_points[:, 2], array_values)
    r, theta = cartesian2polar(x, y) # 極座標に変換
    list_angle, list_score = count_eigen_score(theta, array_values, 5)
    list_score = np.array(list_score)

    attackablity = calc_localizability.global_score_polar(list_score, spoofing_mode)
    list_attackablity.append(attackablity)

    # visualize
    if counter == 0:
        mappable = ax1.scatter(-x, -y, c = eigen_value, cmap = "jet", s = 5)
        ax1.set_xlim(-max_distance, max_distance)
        ax1.set_ylim(-max_distance, max_distance)
        ax1.set_xlabel("x(m)")
        ax1.set_ylabel("y(m)")
        ax1.set_title("Scatter Plot")

        ax2.remove()
        ax2 = plt.subplot(132, projection='polar')
        theta_rad = np.deg2rad(list_angle) 
        R = np.linspace(0, 30, 2)
        C = np.vstack((list_score, list_score))
        
        c = ax2.pcolormesh(theta_rad, R, C, shading='auto', cmap='jet')
        norm = Normalize(vmin = 0, vmax=np.max(C))
        sm = ScalarMappable(cmap='jet', norm=norm)
        sm.set_array([])
        fig.colorbar(sm, ax=ax2, orientation='vertical')

        ax3.plot(list_attackablity, color=(0, 0, 1, 0.3))
        ax3.set_ylabel("attackablity")
        ax3.set_title("Attackablity")

    else:
        ax1.scatter(-x, -y, c = eigen_value, cmap = "jet", s = 5)
        ax1.set_xlim(-max_distance, max_distance)
        ax1.set_ylim(-max_distance, max_distance)
        ax1.set_xlabel("x(m)")
        ax1.set_ylabel("y(m)")
        ax1.set_title("Scatter Plot")

        ax2.remove()
        ax2 = plt.subplot(132, projection='polar')
        theta_rad = np.deg2rad(list_angle) 
        R = np.linspace(0, 30, 2)
        C = np.vstack((list_score, list_score))
        c = ax2.pcolormesh(theta_rad, R, C, shading='auto', cmap='jet')
        norm = Normalize(vmin = 0, vmax=np.max(C))
        sm = ScalarMappable(cmap='jet', norm=norm)
        sm.set_array([])
        fig.colorbar(sm, ax=ax2, orientation='vertical')

        ax3.plot(list_attackablity, color=(0, 0, 1, 0.3))
        ax3.set_ylabel("attackablity")
        ax3.set_title("Attackablity")
        ax3.plot(list_flatten_x, list_flatten, color="red", label="euclid", marker="o")

        if counter % 25 == 0:
            list_flatten_x.append(counter)
            score_average = sum(list_attackablity[-25:]) / 25
            list_flatten.append(score_average)
            ax3.plot(list_flatten_x, list_flatten, color="red", label="euclid", marker="o")

    counter += 1

    fig.tight_layout()
    plt.pause(0.01)

plt.show()