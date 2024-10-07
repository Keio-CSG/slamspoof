#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import tf.transformations as tf

def load_csv(file_name):
    df = pd.read_csv(file_name)
    x, y, z = np.array(df["x"]), np.array(df["y"]), np.array(df["z"])
    qw, qx, qy, qz = np.array(df["qw"]), np.array(df["qx"]), np.array(df["qy"]), np.array(df["qz"])
    return x, y, z, qx, qy, qz, qw

def save_trajectory(traj_x, traj_y, traj_z, save_name):
    df = pd.DataFrame({'x': traj_x, 'y': traj_y, 'z': traj_z})
    df.to_csv(save_name, index=False)

def get_transform_matrix(x, y, z, qx, qy, qz, qw):
    Q = (qx, qy, qz, qw)
    rotation_matrix = tf.quaternion_matrix(Q) # クオータニオンから回転行列に変換

    transform_matrix = np.copy(rotation_matrix) # 並進成分を追加する
    transform_matrix[0, 3] = x # x座標の値
    transform_matrix[1, 3] = y # y座標の値
    transform_matrix[2, 3] = z # z座標の値
    
    return transform_matrix

def initial_transform(map_data, target_data):
    map_x, map_y, map_z, map_qx, map_qy, map_qz, map_qw = load_csv(map_data)
    tar_x, tar_y, tar_z, tar_qx, tar_qy, tar_qz, tar_qw = load_csv(target_data)

    T_map_initial = get_transform_matrix(map_x[0], map_y[0], map_z[0], map_qx[0], map_qy[0], map_qz[0], map_qw[0])
    T_target_initial = get_transform_matrix(tar_x[0], tar_y[0], tar_z[0], tar_qx[0], tar_qy[0], tar_qz[0], tar_qw[0])   

    T_initial_transform = np.linalg.inv(T_map_initial) @ T_target_initial
    return T_initial_transform

def inverse_target_matrix(target_data):
    odom_x, odom_y, odom_z, odom_qx, odom_qy, odom_qz, odom_qw = load_csv(target_data)
    T_odom_zero = get_transform_matrix(odom_x[0], odom_y[0], odom_z[0], odom_qx[0], odom_qy[0], odom_qz[0], odom_qw[0])
    return np.linalg.inv(T_odom_zero)

def convert_target(target_data, initial_transform, initial_target_inverse):
    list_converted_x, list_converted_y, list_converted_z = [], [], []
    x, y, z, qx, qy, qz, qw = load_csv(target_data)

    for i in range(x.shape[0]):
        T_matrix = get_transform_matrix(x[i], y[i], z[i], qx[i], qy[i], qz[i], qw[i]) # T_odom_lidar[i]
        T_map_lidar_i = initial_transform @ initial_target_inverse @ T_matrix # T_map_lidar[i] 
        converted_x, converted_y, converted_z = T_map_lidar_i[0, 3], T_map_lidar_i[1, 3], T_map_lidar_i[2, 3]

        list_converted_x.append(converted_x)
        list_converted_y.append(converted_y)
        list_converted_z.append(converted_z)
    
    return np.array(list_converted_x), np.array(list_converted_y), np.array(list_converted_z)

def plot_traj(ref_x, ref_y, tar_x, tar_y):
    plt.title("Benign-attacked comparison")
    plt.plot(ref_x, ref_y, linestyle="dashed", label="benign-hdl")
    plt.plot(tar_x, tar_y, color="red", linestyle="solid", label="loam-converted")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()

def convert_main(map_data, target_data):
    T_map_lidar_zero = initial_transform(map_data, target_data) # T_map_lidar[0]
    T_odom_lidar_inv = inverse_target_matrix(target_data) # T_odom_lidar[0].inverse()

    x_converted, y_converted, z_converted = convert_target(target_data, T_map_lidar_zero, T_odom_lidar_inv)
    reference_x, reference_y, reference_z, reference_qx, reference_qy, reference_qz, reference_qw = load_csv(map_data)
    plot_traj(reference_x, reference_y, x_converted, y_converted) # 変換後の結果を確認する
    #save_trajectory(x_converted, y_converted, z_converted, save_name)

map_data = "./tum_benign_hdl.csv" # 基準となる地図座標
target_data = "./tum_benign_loam.csv" # 地図に合わせたい座標

convert_main(map_data, target_data)
plt.show()