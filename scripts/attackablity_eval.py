import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os, csv

from evo.core import metrics
from evo.core.units import Unit
from evo.tools import log
log.configure_logging(verbose=False, debug=False, silent=False)

import pprint
from evo.tools import plot
from evo.tools.settings import SETTINGS
SETTINGS.plot_usetex = False

plot.apply_settings(SETTINGS)

from evo.tools import file_interface
from evo.core import sync

def get_attackablity(file_name, duration=485):
    df = pd.read_csv(file_name)
    attackablity = df[df['attackablity'] != 0]["attackablity"]

    df_temp = df[df['attackablity'] != 0]
    timestamp_former = np.array(df_temp[df["timestamp"] < duration/2]["timestamp"])
    timestamp_latter = np.array(df_temp[df["timestamp"] >= duration/2]["timestamp"])

    if timestamp_former.shape[0] != 0 and timestamp_latter.shape[0] != 0:
        former_duration = np.max(timestamp_former) - np.min(timestamp_former)
        latter_duration = np.max(timestamp_latter) - np.min(timestamp_latter)
        spoofing_duration = former_duration + latter_duration
    
    elif timestamp_former.shape[0] != 0 and timestamp_latter.shape[0] == 0:
        spoofing_duration = np.max(timestamp_former) - np.min(timestamp_former)
    
    elif timestamp_former.shape[0] == 0 and timestamp_latter.shape[0] != 0:
        spoofing_duration = np.max(timestamp_latter) - np.min(timestamp_latter)

    return np.mean(np.array(attackablity))

def get_xy(csv_name):
    df = pd.read_csv(csv_name)
    odom_x, odom_y = df["x"], df["y"]
    return np.array(odom_x), np.array(odom_y)

def eval_yerror(ref_y, est_y):
    yerror = abs(est_y[-1] - ref_y[-1])
    return yerror

def load_traj(ref_file, est_file):
    max_diff = 0.01
    traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)

    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)
    traj_est.align(traj_ref, correct_scale=False, correct_only_scale=False)
    return traj_ref, traj_est

def get_ape(traj_ref, traj_est):
    #pose_relation = metrics.PoseRelation.translation_part
    #pose_relation = metrics.PoseRelation.rotation_part
    #pose_relation = metrics.PoseRelation.full_transformation
    pose_relation = metrics.PoseRelation.rotation_angle_deg

    data = (traj_ref, traj_est)
    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data(data)
    ape_stat = ape_metric.get_statistic(metrics.StatisticsType.rmse)
    return ape_stat

def get_rpe(traj_ref, traj_est):
    #pose_relation = metrics.PoseRelation.translation_part
    #pose_relation = metrics.PoseRelation.rotation_part
    #pose_relation = metrics.PoseRelation.full_transformation
    pose_relation = metrics.PoseRelation.rotation_angle_deg

    delta = 1
    delta_unit = Unit.frames
    all_pairs = False
    data = (traj_ref, traj_est)
    rpe_metric = metrics.RPE(pose_relation=pose_relation, delta=delta, delta_unit=delta_unit, all_pairs=all_pairs)
    rpe_metric.process_data(data)
    rpe_stat = rpe_metric.get_statistic(metrics.StatisticsType.max)
    return rpe_stat

def write_csv(attackablity, ape, dist):
    filename = "./dist_filter_injection_ape_rot.csv"
    mode = 'w' if not os.path.exists(filename) else 'a'
    with open(filename, mode, newline='') as csvfile:
        if mode == "w":
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(["attackablity", "ape", "distance"]) 
            #csvwriter.writerow(["attackablity", "rpe", "distance"]) 
            csvwriter.writerow([attackablity, ape, dist]) # timestamp, value
        
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([attackablity, ape, dist])  # timestamp, value

def get_stats(attackablity, ape, rpe):
    # convert to np.array
    attackablity = np.array(attackablity)
    ape = np.array(ape)
    rpe = np.array(rpe)

    ape_1, rpe_1 = ape[(-10000 < attackablity) & (attackablity < -6000)], rpe[(-10000 < attackablity) & (attackablity < -6000)]    
    ape_2, rpe_2 = ape[(-6000 < attackablity) & (attackablity < -3000)], rpe[(-6000 < attackablity) & (attackablity < -3000)] 
    ape_3, rpe_3 = ape[(-3000 < attackablity) & (attackablity < -1000)], rpe[(-3000 < attackablity) & (attackablity < -1000)] 
    ape_4, rpe_4 = ape[(-1000 < attackablity)], rpe[(-1000 < attackablity)] 
    
    # calc stats
    ape_1_mean, ape_1_std = np.mean(ape_1), np.std(ape_1)
    ape_2_mean, ape_2_std = np.mean(ape_2), np.std(ape_2)
    ape_3_mean, ape_3_std = np.mean(ape_3), np.std(ape_3)
    ape_4_mean, ape_4_std = np.mean(ape_4), np.std(ape_4)

    rpe_1_mean, rpe_1_std = np.mean(rpe_1), np.std(rpe_1)
    rpe_2_mean, rpe_2_std = np.mean(rpe_2), np.std(rpe_2)
    rpe_3_mean, rpe_3_std = np.mean(rpe_3), np.std(rpe_3)
    rpe_4_mean, rpe_4_std = np.mean(rpe_4), np.std(rpe_4)

    print("APE")
    print("-10000 < A < -6000:{} ± {}".format(ape_1_mean, ape_1_std))
    print("-6000 < A < -3000:{} ± {}".format(ape_2_mean, ape_2_std))
    print("-3000 < A < -1000:{} ± {}".format(ape_3_mean, ape_3_std))
    print("-1000 < A:{} ± {}\n".format(ape_4_mean, ape_4_std))

    print("RPE")
    print("-10000 < A < -6000:{} ± {}".format(rpe_1_mean, rpe_1_std))
    print("-6000 < A < -3000:{} ± {}".format(rpe_2_mean, rpe_2_std))
    print("-3000 < A < -1000:{} ± {}".format(rpe_3_mean, rpe_3_std))
    print("-1000 < A:{} ± {}".format(rpe_4_mean, rpe_4_std))

if __name__ == '__main__':
    ref_csv = "/home/rokuto/cbulid_ws/src/spoofing_sim/0.0_0.0_loam.csv"
    ref_file = "/home/rokuto/cbulid_ws/src/spoofing_sim/tum_0.0_0.0_loam.txt"

    global_est_dir = "/home/rokuto/cbulid_ws/src/spoofing_sim/Trajectory_loam_noise"
    global_est_attackablity = "/home/rokuto/cbulid_ws/src/spoofing_sim/Attackablity_loam_noise"

    est_dir = "/home/rokuto/cbulid_ws/src/spoofing_sim/Trajectory_e2e_test"
    attackablity_dir = "/home/rokuto/cbulid_ws/src/spoofing_sim/Attackablity_e2e_test"
    filter_distance = 100

    ref_x, ref_y = get_xy(ref_csv)

    list_ape = []
    list_attackablity = []
    list_rpe = []

    list_ref_ape = []
    list_ref_rpe = []
    list_ref_attackablity = []

    list_error = [] # 終点の座標

    est_file_list = os.listdir(est_dir)
    attackablity_file_list = os.listdir(attackablity_dir)

    est_file_list.sort()
    attackablity_file_list.sort()

    global_est_file_list = os.listdir(global_est_dir)
    global_attackablity_file_list = os.listdir(global_est_attackablity)

    est_file_list.sort()
    attackablity_file_list.sort()

    for est, attackablity in zip(est_file_list, attackablity_file_list):
        est_file = est_dir + "/" + est
        attackablity_file = attackablity_dir + "/" + attackablity

        print(est)

        traj_ref, traj_est = load_traj(ref_file, est_file)
        ape_value = get_ape(traj_ref, traj_est)
        list_ape.append(ape_value)
        print("APE:{}m".format(round(ape_value, 2)))

        attackablity_value = get_attackablity(attackablity_file)
        list_attackablity.append(attackablity_value)
        #list_duration.append(duration_value)
        print("Attackablity:{}".format(round(attackablity_value, 2)))

        rpe = get_rpe(traj_ref, traj_est)
        print("RPE:{}m\n".format(round(rpe, 2)))
        list_rpe.append(rpe)

        #x, y = get_xy(attackablity_file)
        #error = ((x[-1] - ref_x[-1]) ** 2 + (y[-1] - ref_y[-1]) ** 2) ** 0.5
        #list_error.append(error)

        # csvに書き出し
        #write_csv(attackablity_value, ape_value, filter_distance)
        #write_csv(attackablity_value, rpe, filter_distance)
    
    for ref_est, ref_attackablity in zip(global_est_file_list, global_attackablity_file_list):
        est_file = global_est_dir  + "/" + ref_est
        attackablity_file = global_est_attackablity + "/" + ref_attackablity

        attackablity_value = get_attackablity(attackablity_file)
        list_ref_attackablity.append(attackablity_value)

        traj_ref, traj_est = load_traj(ref_file, est_file)
        ape_value = get_ape(traj_ref, traj_est)
        list_ref_ape.append(ape_value)

        rpe = get_rpe(traj_ref, traj_est)
        list_ref_rpe.append(rpe)
        
        print(ref_est)
        print("APE:{}m".format(ape_value))
    
    get_stats(list_attackablity, list_ape, list_rpe)

    plt.style.use('default')

    fig = plt.figure(figsize=(12, 6))
    fig.suptitle("HFR rotation error")
    
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.scatter(list_ref_attackablity, list_ref_ape)
    ax1.scatter(list_attackablity, list_ape, color="red")
    ax1.set_xlabel("Attackablity")
    ax1.set_ylabel("APE (rmse) (deg)")
    ax1.set_title("Attackablity-APE")
    
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.scatter(list_attackablity, list_rpe)
    ax2.set_xlabel("Attackablity")
    ax2.set_ylabel("RPE (max) (deg)")
    ax2.set_title("Attackablity-RPE")

    """
    ax3= fig.add_subplot(1, 3, 3)
    ax3.scatter(list_attackablity, list_rpe)
    ax3.set_xlabel("Attackablity")
    ax3.set_ylabel("yerror (m)")
    ax3.set_title("Attackablity-yerror")
    """

    plt.subplots_adjust(wspace=0.3)
    plt.show()
