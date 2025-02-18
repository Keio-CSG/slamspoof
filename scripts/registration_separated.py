#!/usr/bin/env python3

import numpy as np
import pandas as pd
import small_gicp
import sys

def random_sampling(array, sample_rate):

    num_sample = int(array.shape[0] * sample_rate)

    indices_1 = np.random.choice(array.shape[0], num_sample, replace=False)
    sampled_array1 = array[indices_1]

    indices_2 = np.random.choice(array.shape[0], num_sample, replace=False)
    sampled_array2 = array[indices_2]

    return sampled_array1, sampled_array2

def points_noise(array, scale_translation):
    rng = np.random.default_rng()

    noise_x = rng.normal(0, scale_translation, array.shape[0])
    noise_y = rng.normal(0, scale_translation, array.shape[0])
    noise_z = rng.normal(0, scale_translation, array.shape[0])

    array_noised = array.copy()
    array_noised[:, 0] += noise_x
    array_noised[:, 1] += noise_y
    array_noised[:, 2] += noise_z

    return array_noised

def calc_localizability(hessian_matrix):
    cov_matrix = np.linalg.pinv(hessian_matrix) 

    cov_eigen_value, cov_eigen_vector = np.linalg.eig(cov_matrix)
    localizability_zhen = np.min(cov_eigen_value)

    localizability_kondo = np.linalg.det(cov_matrix) ** 0.5

def calc_factor(source_points, target_points, localizability):

    source, source_tree = small_gicp.preprocess_points(source_points, downsampling_resolution=0.3)
    target, target_tree = small_gicp.preprocess_points(target_points, downsampling_resolution=0.3)

    result = small_gicp.align(target, source, target_tree)
    result = small_gicp.align(target, source, target_tree, result.T_target_source)

    factors = [small_gicp.GICPFactor()]
    rejector = small_gicp.DistanceRejector()

    sum_H = np.zeros((6, 6))
    sum_b = np.zeros(6)
    sum_e = 0.0

    hessian = result.H 

    if localizability:
        calc_localizability(hessian)
    else:
        pass

    hessian_rotation = hessian[0:3, 0:3]
    hessian_translation = hessian[3:6, 3:6]

    rot_eigen_value, rot_eigen_vector = np.linalg.eig(hessian_rotation)
    trans_eigen_value, trans_eigen_vector = np.linalg.eig(hessian_translation)

    rot_global_min_value = np.argmin(rot_eigen_value)
    trans_global_min_value = np.argmin(trans_eigen_value)

    rot_global_min_vector = rot_eigen_vector[:, rot_global_min_value]
    trans_global_min_vector = trans_eigen_vector[:, trans_global_min_value]

    list_xyz = []
    list_cov_eigen_value = []

    for i in range(source.size()):
        succ, H, b, e = factors[0].linearize(target, source, target_tree, result.T_target_source, i, rejector)
        if succ:

            rot_point_eigen_value, rot_point_eigen_vector = np.linalg.eig(H[0:3, 0:3]) 
            trans_point_eigen_value, trans_point_eigen_vector = np.linalg.eig(H[3:6, 3:6]) 

            rot_local_max_value = np.argmax(rot_point_eigen_value) 
            trans_local_max_value = np.argmax(trans_point_eigen_value)

            rot_local_max_vector = rot_point_eigen_vector[:, rot_local_max_value] 
            trans_local_max_vector =trans_point_eigen_vector[:, trans_local_max_value] 
            
            dp_rot = np.dot(rot_global_min_vector, rot_local_max_vector)
            dp_trans = np.dot(trans_global_min_vector, trans_local_max_vector)

            list_cov_eigen_value.append(abs(dp_trans)) 
            list_xyz.append(source.points()[i, 0:3])

            sum_H += H
            sum_b += b
            sum_e += e
    
    return np.array(list_xyz), np.array(list_cov_eigen_value)

def execute_gicp(array, num_iteration, sample_rate, scale_translation):
    counter = 1
    pc1, pc2 = random_sampling(array, sample_rate)

    while counter <= int(num_iteration):

        if counter == 1:
            source, target = pc1, points_noise(pc2, scale_translation)
            coordinate_origin, dot_eigen_value_origin = calc_factor(source, target, localizability=False)
            counter += 1

        else:
            source, target = pc1, points_noise(pc2, scale_translation)

            if counter == num_iteration:
                coordinate, dot_eigen_value = calc_factor(source, target, localizability=False)
            else:
                 coordinate_origin, dot_eigen_value_origin = calc_factor(source, target, localizability=False)
                 
            if coordinate_origin.shape == coordinate.shape:
                coordinate_origin = (coordinate_origin + coordinate) / 2
            else:
                pass

            if dot_eigen_value_origin.shape == dot_eigen_value.shape:
                dot_eigen_value_origin = (dot_eigen_value_origin + dot_eigen_value) / 2
            else:
                pass
            counter += 1
    
    return coordinate_origin, dot_eigen_value_origin

