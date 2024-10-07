#!/usr/bin/env python3

import os, re, random
import rospy
import pandas as pd
import numpy as np
import matplotlib 
import seaborn as sns
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

def output_heatmap(x, y, max_distance, score_array, num_x, num_y): # x座標, y座標, score配列, x軸分割数, y軸分割数
    # 参照する範囲
    # x_range = np.max(x) - np.min(x)
    # y_range = np.max(y) - np.min(y)
    x_range = 2 * max_distance
    y_range = 2 * max_distance
    x_step, y_step = x_range / num_x, y_range / num_y
    score_flatten = score_array[:, 0]
    table = np.vstack([x, y, score_flatten]).T
    score_grid = np.zeros((num_x, num_y)) 

    for ix in range(num_x):
        for iy in range(num_y):
            #left, right = np.min(x) + x_step * ix, np.min(x) + x_step * (ix + 1)
            left, right = -max_distance + x_step * ix, -max_distance + x_step * (ix + 1)

            #under, upper = np.min(y) + y_step * iy, np.min(y) + y_step * (iy + 1)
            under, upper = -max_distance + y_step * iy, -max_distance + y_step * (iy + 1)
            grid_in_points = table[((left <= table[:, 0]) & (table[:, 0] < right)) & ((under <= table[:, 1]) & (table[:, 1] < upper))]

            if grid_in_points.shape[0] == 0:
                score_grid[ix, iy] = 0
            else:
                # mean
                # score_grid[ix, iy] = np.mean(grid_in_points[:, 2])

                # sum
                score_grid[ix, iy] = np.sum(grid_in_points[:, 2])

                # exp
                #score_grid[ix, iy] = np.sum(np.exp(grid_in_points[:, 2]) - 1)

                # exp2
                """
                score_grid[ix, iy] = np.sum((np.exp(2 * grid_in_points[:, 2] - (np.e - 1))) / 2)

                if score_grid[ix, iy] < 0:
                    score_grid[ix, iy] = 0
                else:
                    pass
                """

    return score_grid

dir_name = rospy.get_param('directory_name', '/home')
max_distance = float(rospy.get_param('max_distance', 10))
z_min, z_max = float(rospy.get_param('z_min', -1)), float(rospy.get_param('z_max', 1))
num_x_divide, num_y_divide = int(rospy.get_param('x_divide', 10)), int(rospy.get_param('y_divide', 10))
FILES = sort_files(dir_name)
list_attackablity_euclid = []
list_attackablity_manhattan = []

list_flatten_x = [] #平滑化した横軸
list_flatten = [] #平滑化した縦軸
ymax = calc_ylim(FILES, dir_name)

# set gigure
fig, axes = plt.subplots(1, 3, figsize=(15, 6))
plt.subplots_adjust(wspace=0.5)
counter = 0

for file in FILES:
    axes[0].clear()
    axes[1].clear()
    axes[2].clear()
    path = dir_name + "/" + str(file)
    array_points, array_values = csv_2_array(path)

    # preprocess to visualize
    x, y, z, eigen_value = downsample_visualize(array_points[:, 0], array_points[:, 1], array_points[:, 2], array_values)
    list_angle, list_score = count_eigen_score(calc_angle(array_points), array_values, 10) # 角度情報, 重要度情報, 水平方位角範囲の広さ(deg)

    score_table = output_heatmap(x, y, max_distance, eigen_value, num_x_divide, num_y_divide)
    attackablity_euclid, attackability_manhattan = calc_localizability.global_score(score_table)
    list_attackablity_euclid.append(attackablity_euclid)
    list_attackablity_manhattan.append(attackability_manhattan)

    # visualize
    if counter == 0:
        mappable = axes[0].scatter(y, -x, c = eigen_value, cmap = "jet", s = 5)
        axes[0].set_xlim(-max_distance, max_distance)
        axes[0].set_ylim(-max_distance, max_distance)
        axes[0].set_xlabel("x(m)")
        axes[0].set_ylabel("y(m)")
        axes[0].set_title("Scatter Plot")

        sns.heatmap(score_table, vmax = np.max(score_table), vmin = np.min(score_table), center=0, cmap='jet', annot=True, xticklabels=False, yticklabels=False, annot_kws={"size": 5}, fmt='.1f', ax=axes[1])
        axes[1].set_title("Attackablity:{}".format(round(attackablity_euclid, 3)))

        axes[2].plot(list_attackablity_euclid, color=(0, 0, 1, 0.3), label="euclid")
        #axes[2].plot(list_attackablity_manhattan, marker='o', color="orange", label="manhattan")
        axes[2].set_ylabel("attackablity")
        axes[2].set_title("Attackablity")
        axes[2].legend()

    else:
        axes[0].scatter(y, -x, c = eigen_value, cmap = "jet", s = 5)
        axes[0].set_xlim(-max_distance, max_distance)
        axes[0].set_ylim(-max_distance, max_distance)
        axes[0].set_xlabel("x(m)")
        axes[0].set_ylabel("y(m)")
        axes[0].set_title("Scatter Plot")

        sns.heatmap(score_table, cbar=False, center=0, cmap='jet', annot=True, xticklabels=False, yticklabels=False, annot_kws={"size": 5}, fmt='.1f', ax=axes[1])
        axes[1].set_title("Attackablity:{}".format(round(attackablity_euclid, 3)))

        if counter % 25 == 0:
            list_flatten_x.append(counter)
            score_average = sum(list_attackablity_euclid[-25:]) / 25
            list_flatten.append(score_average)
            axes[2].plot(list_flatten_x, list_flatten, color="red", label="euclid", marker="o")

        axes[2].plot(list_attackablity_euclid, color=(0, 0, 1, 0.3), label="euclid")
        axes[2].plot(list_flatten_x, list_flatten, color="red", label="euclid", marker="o")
        #axes[2].plot(list_attackablity_manhattan, marker='o', color="orange", label="manhattan")
        axes[2].set_ylabel("attackablity")
        axes[2].set_title("Attackablity")

    counter += 1

    fig.tight_layout()
    plt.pause(0.01)

plt.show()
