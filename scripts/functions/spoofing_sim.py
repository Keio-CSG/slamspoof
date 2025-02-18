#!/usr/bin/env python3

import numpy as np
import rospy

def cartesian2polar(x, y):
    r = (x ** 2 + y ** 2) ** 0.5
    theta = np.degrees(np.arctan2(y, x)) 
    return r, theta

def polar2cartesian(r, theta):
    x = r * np.cos(np.radians(theta))
    y = r * np.sin(np.radians(theta))
    return x, y

def removal_simulation(raw_points, mask_index):
    removed_points = np.delete(raw_points, list(mask_index[0]))
    return removed_points

def set_distance(timestamp):
    minimum_distance = float(rospy.get_param('wall_distance_min', '0.0'))
    maximum_distance = float(rospy.get_param('wall_distance_max', '0.0'))
    time_cycle = float(rospy.get_param('spoofing_cycle', '1.0'))

    f_t = (((maximum_distance - minimum_distance) / time_cycle) * (timestamp % time_cycle)) + minimum_distance
    return f_t

def noise_simulation(raw_points, largest_score_angle, spoofing_range):
    rng = np.random.default_rng() 
    horizontal_resolution = float(rospy.get_param('lidar_horizontal_resolution', '0.1'))
    vertical_lines = float(rospy.get_param("lidar_vertical_lines", "16"))
    spoofing_rate = float(rospy.get_param("spoofing_success_rate", "0.1"))

    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 
    if temp_min < 0: 
        min = 360 - temp_min
        max = temp_max

    elif temp_max > 360: 
        min = temp_min
        max = temp_max - 360

    else: 
        min = temp_min
        max = temp_max

    r, theta = cartesian2polar(raw_points[:, 0], raw_points[:, 1]) 
    z = raw_points[:, 2]
    mask = ((min <= theta) & (theta <= max)) 

    r_deleted = r[~mask]
    theta_deleted = theta[~mask]
    z_deleted = z[~mask]

    num_spoofed_points = int((spoofing_range / horizontal_resolution) * vertical_lines * spoofing_rate)

    r_noise = rng.uniform(0.0, 50.0, num_spoofed_points)
    theta_noise = rng.uniform(temp_min, temp_max, num_spoofed_points)
    z_noise = r_noise * np.sin(np.degrees(rng.uniform(-15.0, 15.0, num_spoofed_points)))

    r_spoofed = np.concatenate((r_deleted, r_noise))
    theta_spoofed = np.concatenate((theta_deleted, theta_noise))
    z_spoofed = np.concatenate((z_deleted, z_noise))

    x_spoofed, y_spoofed = polar2cartesian(r_spoofed, theta_spoofed)
    spoofed_points = np.vstack((x_spoofed, y_spoofed, z_spoofed)).T

    return spoofed_points 

def injection_simulation(raw_points, largest_score_angle, spoofing_range, injection_dist):
    rng = np.random.default_rng()

    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 
    if temp_min < 0: 
        min = 360 - temp_min
        max = temp_max

    elif temp_max > 360: 
        min = temp_min
        max = temp_max - 360

    else: 
        min = temp_min
        max = temp_max

    r, theta = cartesian2polar(raw_points[:, 0], raw_points[:, 1])
    z = raw_points[:, 2]
    mask = ((min <= theta) & (theta <= max)) 

    r_deleted = r[~mask]
    theta_deleted = theta[~mask]
    z_deleted = z[~mask]

    horizontal_resolution = float(rospy.get_param('lidar_horizontal_resolution', '0.1'))
    vertical_lines = float(rospy.get_param("lidar_vertical_lines", "16"))
    n_injection = int((spoofing_range / horizontal_resolution) * vertical_lines)  
    #vertical_angle_canditate = [-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15] 
    vertical_angle_canditate = [-1.333, -1.0, -0.667, -0.333, 0, 0.333, 0.667, 1.0, 1.333]

    r_wall = np.full(n_injection, injection_dist)
    theta_wall = rng.uniform(temp_min, temp_max, n_injection)
    vertical_angle_wall = np.random.choice(vertical_angle_canditate, size=n_injection, replace=True)
    z_wall = r_wall * np.sin(np.degrees(vertical_angle_wall))

    r_spoofed = np.concatenate((r_deleted, r_wall))
    theta_spoofed = np.concatenate((theta_deleted, theta_wall))
    z_spoofed = np.concatenate((z_deleted, z_wall))

    x_spoofed, y_spoofed = polar2cartesian(r_spoofed, theta_spoofed)
    spoofed_points = np.vstack((x_spoofed, y_spoofed, z_spoofed)).T
   
    return spoofed_points 
    

def decide_mask(horizontal_angle, largest_score_angle, spoofing_range):
    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 
    if temp_min < 0: 
        min = 360 - temp_min
        spoofing_condition = ((min <= horizontal_angle) | (horizontal_angle <= temp_max))

    elif temp_max > 360: 
        max = temp_max - 360
        spoofing_condition = ((temp_min <= horizontal_angle) | (horizontal_angle <= max))

    else: 
        spoofing_condition = ((temp_min <= horizontal_angle) & (horizontal_angle <= temp_max))
    
    return spoofing_condition

def spoof_main(pointcloud, largest_score_angle, spoofing_range): 
    angle = np.degrees(np.arctan2(pointcloud[:, 1], pointcloud[:, 0])) + 180
    mask_condition = decide_mask(angle, largest_score_angle, spoofing_range)
    mask_index = np.where(mask_condition)

    x_spoofed , y_spoofed, z_spoofed = pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2]

    spoofed_points = noise_simulation(pointcloud, largest_score_angle, spoofing_range)
   
    x_spoofed, y_spoofed, z_spoofed = spoofed_points[:, 0], spoofed_points[:, 1], spoofed_points[:, 2]

    return x_spoofed, y_spoofed, z_spoofed
    
def injection_main(pointcloud, largest_score_angle, spoofing_range, wall_dist):
    spoofed_points = injection_simulation(pointcloud, largest_score_angle, spoofing_range, wall_dist)
    x_spoofed, y_spoofed, z_spoofed = spoofed_points[:, 0], spoofed_points[:, 1], spoofed_points[:, 2]
    return x_spoofed, y_spoofed, z_spoofed

def dynemic_injection_main(pointcloud, timestamp, largest_score_angle, spoofing_range):
    wall_dist = set_distance(timestamp)
    spoofed_points = injection_simulation(pointcloud, largest_score_angle, spoofing_range, wall_dist)
    x_spoofed, y_spoofed, z_spoofed = spoofed_points[:, 0], spoofed_points[:, 1], spoofed_points[:, 2]
    return x_spoofed, y_spoofed, z_spoofed