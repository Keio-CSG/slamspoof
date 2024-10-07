#!/usr/bin/env python3

import numpy as np

def cartesian2polar(x, y):
    r = (x ** 2 + y ** 2) ** 0.5
    theta = np.degrees(np.arctan2(y, x)) + 180
    return r, theta

def count_eigen_score(angle_array, eigen_array, step):
    list_angle, list_score = [], []
    for i in range(int(360 / step)):
        mask = ((angle_array >= step * i) & (angle_array < step * (i+1)))
        score_table = np.sum(eigen_array[mask])
        list_score.append(score_table)
        list_angle.append(step * (i + 0.5))
    
    return list_angle, list_score

def fd(distance, threshold):
    return -distance + threshold

def calc_reward(score, distance):
    reward = score * fd(distance, threshold=4)
    return reward

def calc_reward_polar(score, distance, threshold):
    reward = score * fd(distance, threshold)
    return reward

def calc_distance_polar(index, index_ref, num_of_indexes):
    index_diff1 = abs(index - index_ref)
    index_diff2 = num_of_indexes - index_diff1
    return min(index_diff1, index_diff2)

def global_score(score_table):
    flat_indices = np.argsort(score_table, axis=None)[::-1]
    sorted_indices = np.unravel_index(flat_indices, score_table.shape)
    sorted_indices_list = list(zip(sorted_indices[0], sorted_indices[1]))

    counter = 0
    largest_score, largest_index = 0, [0, 0]
    localizability_euclid = 0
    localizability_manhattan = 0

    for index in sorted_indices_list:
        if counter == 0: # largest score
            largest_score = score_table[index[0], index[1]]
            largest_index[0], largest_index[1] = index[0], index[1]
            localizability_euclid += calc_reward(largest_score, 0)
            localizability_manhattan += calc_reward(largest_score, 0)
        else:
            point_score = score_table[index[0], index[1]]

            # ユークリッド距離
            dist_x, dist_y = largest_index[0] - index[0], largest_index[1] - index[1]
            dist_euclud = (dist_x ** 2 + dist_y ** 2) ** 0.5

            # マンハッタン距離
            dist_xm, dist_ym = abs(largest_index[0] - index[0]), abs(largest_index[1] - index[1])
            dist_m = dist_xm + dist_ym
            reward_euclid = calc_reward(point_score, dist_euclud)
            reward_manhattan = calc_reward(point_score, dist_m)
            localizability_euclid += reward_euclid
            localizability_manhattan += reward_manhattan
        
        counter += 1

    return localizability_euclid, localizability_manhattan

def global_score_polar(score, spoofing_mode):
    sorted_index = np.argsort(-score)
    sorted_score = score[sorted_index]

    counter = 0
    localizability = 0

    largest_score, largest_index = sorted_score[0], sorted_index[0] # 最大スコアと対応するインデックス

    for index in sorted_index:
        distance = calc_distance_polar(index, largest_index, sorted_index.shape[0])
        score_local = score[index]
        if spoofing_mode == "HFR":
            localizability += calc_reward_polar(score_local, distance, threshold=8) # HFR:threshold=8 A-HFR:threshold=2
        elif spoofing_mode == "AHFR":
            localizability += calc_reward_polar(score_local, distance, threshold=2)

    return localizability