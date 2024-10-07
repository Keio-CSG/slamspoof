#!/usr/bin/env python3

import pandas as pd
import numpy as np
import rospy
import matplotlib.pyplot as plt

def load_csv(file_name):
    df = pd.read_csv(file_name)
    x, y, z = np.array(df['x']), np.array(df['y']), np.array(df['z'])
    vec_x, vec_y = np.array(df['vec_x']), np.array(df['vec_y'])
    smvs = np.array(df['smvs'])

    return x, y, z, vec_x, vec_y, smvs

def ccw(A, B, C):
    return np.cross(B - A, C - A) > 0

def do_lines_intersect(p1, p2, p3, p4):

    p1, p2, p3, p4 = map(np.array, (p1, p2, p3, p4))
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

def cross_point(p1, p2, p3, p4):
    p1, p2, p3, p4 = map(np.array, (p1, p2, p3, p4))
    AB = p2 - p1
    CD = p4 - p3

    cross = np.cross(AB, CD)
    if cross == 0:
        return None
    else:
        # tとuの計算
        t = np.cross(p3 - p1, CD) / cross
        u = np.cross(p3 - p1, AB) / cross

        # 交点が線分上にあるかの判定
        if 0 <= t <= 1 and 0 <= u <= 1:
            intersection = p1 + t * AB

        return intersection
    
def rotation(list_x, list_y, rotate_angle, traj_center): # rotate_angle : deg
    temp_x = np.array(list_x) - traj_center[0]
    temp_y = np.array(list_y) - traj_center[1]

    points = np.vstack((np.array(temp_x), np.array(temp_y))).T

    rotation_matrix = np.array([[np.cos(np.radians(rotate_angle)), -np.sin(np.radians(rotate_angle))], [np.sin(np.radians(rotate_angle)), np.cos(np.radians(rotate_angle))]])
    rotated_points = np.dot(points, rotation_matrix)

    rotated_x, rotated_y = rotated_points[:, 0], rotated_points[:, 1]

    return rotated_x, rotated_y

def reflect_over_line(temp_x, temp_y,  a, b):

    points = np.vstack((np.array(temp_x), np.array(temp_y))).T
    reflected_points = []
    
    for point in points:
        x, y = point

        px = (x + a * (y - b)) / (a**2 + 1)
        py = (a * x + (a**2) * y + b) / (a**2 + 1)
        
        x_reflected = 2 * px - x
        y_reflected = 2 * py - y
        
        reflected_points.append([x_reflected, y_reflected])
    
    return np.array(reflected_points)

def main(filename):
    x, y, z, vec_x, vec_y, smvs = load_csv(filename)
    n_percentile = 3 #上位n%
    r = 200 # 終点の座標計算に使用

    list_intersection_x = []
    list_intersection_y = []

    list_canditate_x, list_canditate_y = [], []

    n = int(x.shape[0] * (n_percentile/100))
    indices = np.argsort(smvs)[-n:][::-1]

    x_extracted = x[indices]
    y_extracted = y[indices]
    z_extracted = z[indices]
    vec_x_extracted = vec_x[indices]
    vec_y_extracted = vec_y[indices]

    dst_x = x_extracted + r * vec_x_extracted
    dst_y = y_extracted + r * vec_y_extracted

    for i in range(x_extracted.shape[0]):
        ref_x, ref_y = x_extracted[i], y_extracted[i]
        ref_sp = (ref_x, ref_y) 

        ref_dst_x, ref_dst_y = dst_x[i], dst_y[i]
        ref_dp = (ref_dst_x, ref_dst_y) 

        for j in range(x_extracted.shape[0]):
            if i != j:
                tar_x, tar_y = x_extracted[j], y_extracted[j]
                tar_sp = (tar_x, tar_y) 
                tar_dst_x, tar_dst_y = dst_x[j], dst_y[j]
                tar_dp = (tar_dst_x, tar_dst_y) 

                if do_lines_intersect(ref_sp, ref_dp, tar_sp, tar_dp):
                    intersection = cross_point(ref_sp, ref_dp, tar_sp, tar_dp)
                    list_intersection_x.append(intersection[0])
                    list_intersection_y.append(-intersection[1])

                    list_canditate_x.append(tar_x)
                    list_canditate_y.append(tar_y)

    intersection_x = np.array(list_intersection_x)
    intersection_y = np.array(list_intersection_y)

    mean_x = np.mean(intersection_x)
    std_x = np.std(intersection_x)
    temp_x = intersection_x[np.abs(intersection_x - mean_x) < 2 * std_x]
    x_center = (np.max(temp_x) + np.min(temp_x)) / 2

    mean_y = np.mean(intersection_y)
    std_y = np.std(intersection_y)
    temp_y = intersection_y[np.abs(intersection_x - mean_x) < 2 * std_x]
    y_center = (np.max(temp_y) + np.min(temp_y)) / 2

    traj_approx_line = np.polyfit(list_canditate_x, list_canditate_y, 1)
    f_traj = np.poly1d(traj_approx_line)

    traj_center = (sum(list_canditate_x)/len(list_canditate_x), sum(list_canditate_y)/len(list_canditate_y))

    traj_angle = np.degrees(np.arctan(traj_approx_line[0]))

    rotated_x, rotated_y = rotation(list_intersection_x, list_intersection_y, traj_angle+180, traj_center)

    katamuki = -1 / traj_approx_line[0]
    seppen = x_center/traj_approx_line[0] + y_center

    f_spoofer = np.poly1d(np.array([katamuki, seppen]))

    plot_x = [0 + 1*i for i in range(30)]

    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(1, 1, 1)
    scatter = plt.scatter(x, y, c = smvs, cmap='jet')
    plt.colorbar(scatter, label='SMVS')
    #ax1.plot(plot_x, f_spoofer(plot_x))
    ax1.scatter(temp_x, temp_y, marker="x", color="red", label="important object canditate")
    ax1.set_title("decide spoofer location")
    ax1.set_xlabel("x", fontsize=15)
    ax1.set_ylabel("y", fontsize=15)
    ax1.legend()

    plt.show()

if __name__ == '__main__':
    filename = rospy.get_param('vulnerablity_file_path', '/home/')
    main(filename)