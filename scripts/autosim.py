
import subprocess
import time
import os
import signal
import numpy as np

def run_roslaunch_and_check_rosbag(rospkg, launch_file, bag_file, x, y):
    # roslaunchを開始する
    launch_process = subprocess.Popen(["roslaunch", rospkg, launch_file, f"x:={x}", f"y:={y}"], preexec_fn=os.setsid)
    time.sleep(0.1)  # rosbagの再生が始まるまで待機（必要に応じて調整）

    # rosbag再生プロセスを取得する
    bag_process = subprocess.Popen(['rosbag', 'play', bag_file])

    # rosbagの再生が終了するのを待機する
    bag_process.wait()

    # roslaunchを終了する
    os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
    launch_process.terminate()
    launch_process.wait()

rospkg_name = "spoofing_sim"
launch_file = "sim3_test.launch"
bagfile = "/home/rokuto/07_03.bag"

"""
num_simulations = 100
rng = np.random.default_rng()

x_canditate = rng.integers(-10.0, 80.0, num_simulations)
y_canditate = x_canditate * rng.uniform(-1, 0.5) + rng.normal(0, 1, num_simulations)
"""
num_simulations = 3
rng = np.random.default_rng()
#base_x_canditate = [-5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90] 
#base_y_canditate = [-10, -10, -10, -10, -10, -10, -15, -15, -15, -20, -20, -20, -30, -30, -30, -30] 
base_x_canditate = [10]
base_y_canditate = [22.4]

x_canditate = []
y_canditate = []

#noise = [0.05, -0.05, 0.1, -0.1, 0] #かぶり防止
noise_x, noise_y = np.array(rng.normal(0, 0.01, num_simulations)), np.array(rng.normal(0, 0.01, num_simulations))
for nx, ny in zip(noise_x, noise_y):
    #new_x_canditate = list(np.array(base_x_canditate) + n)
    #new_y_canditate = list(np.array(base_y_canditate) + n)
    new_x_canditate = float(base_x_canditate[0] + nx)
    new_y_canditate = float(base_y_canditate[0] + ny)

    #x_canditate.extend(new_x_canditate)
    #y_canditate.extend(new_y_canditate)
    x_canditate.append(new_x_canditate)
    y_canditate.append(new_y_canditate)

if len(x_canditate) == len(y_canditate):
    print(len(x_canditate))

    for x, y in zip(x_canditate, y_canditate):
        run_roslaunch_and_check_rosbag(rospkg_name, launch_file, bagfile, x, y)
        time.sleep(1)  # 次のroslaunchを開始する前に少し待機（必要に応じて調整）

else:
    print(len(x_canditate), len(y_canditate))
