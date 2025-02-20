# Slamspoof

Codes for (ICRA'25) "SLAMSpoof: Practical LiDAR Spoofing Attacks on Localization Systems Guided by Scan Matching Vulnerability Analysis"

[Paper Link](https://arxiv.org/abs/2502.13641)

# What is Slamspoof ? 
Slamspoof is a framework that analyzes LiDAR SLAM and reveals vulnerabilities to adversarial laser pulse injections, or LiDAR spoofing. The vulnerabilities are quantitatively evaluated using the SMVS (Scan Matching Vulnerability Score) metric, and the values can be referenced across the entire map. Please note that this repository focuses on localization using LiDAR-based localization (Visual Odometry is not covered, sorry). Additionally, the framework supports kiss-icp and A-LOAM as localization methods.

# Related repositories
- [evo](https://github.com/MichaelGrupp/evo)
  - The output results are generated in a format that can be evaluated using evo.
- [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)
  - The “main” branch supports KISS-ICP, the A-LOAM version is available in the “loam” branch
# Prerequisites
- ROS noetic
  - Installation is [here](https://wiki.ros.org/noetic/Installation/Ubuntu) (ROS wiki)
  - Workspace should be built catkin build.
    ```
    sudo apt-get install python3-catkin-tools
    source /opt/ros/noetic/setup.bash
    mkdir -p ~/your_ws/src
    cd ~/your_ws
    catkin build
    source devel/setup.bash
    ```
- kiss-icp
  - Install [here](https://github.com/PRBonn/kiss-icp)

- small_gicp
  - Install [here](https://github.com/koide3/small_gicp). We recommend installation from source.

- Python packages
  - numpy
  - pandas
  - matplotlib
  - rosbags
      
# Install Slamspoof
First, confirm prerequisites are correctly installed.
```
# for ROS noetic
cd your_ros_workspace/src
git clone https://github.com/ngtroku/slamspoof.git
cd .. && catkin build 
source devel/setup.bash
```

# How to use
- This repository has two functions
    - Calculate SMVS and visualization
    - Simulation of LiDAR Spoofing Attacks
## How to run SLAM and calculate SMVS
  ```
  roslaunch slamspoof run_slam.launch
  ```
And run rosbag in another terminal
  ```
  rosbag play (your_rosbag).bag
  ```
### Settings 
- Line 3,4,5: Directory path to save simulation results.
  - Line 3: `name_smvs_dir` is path for storing SMVS data
  - Line 4: `name_vul_dir` is path for Vulnerable direction of SLAM system
  - Line 5: `name_traj_dir` is path storing SLAM results
- Line 8 `subscribe_topic_name` : Name of point cloud rostopic to input into SLAM system
- Line 15 `lidar_topic_length` : PointCloud topic format
  - If you use Velodyne LiDAR, set value 22
  - If you use Livox LiDAR, set value 18
  - If you use Pointcloud topic edited rosbag_editer.launch, set value 12

## How to visualize SMVS
```
roslaunch slamspoof visualizer.launch
```
### Setting
- Line2 `smvs_file_path`:  Directory of SMVS data

## How to simulate LiDAR Spoofing Attacks
Normal Trajectory must be obtained before simulating attack.
```
roslaunch slamspoof rosbag_editer.launch
```
Simulation settings are changed from config.json

### Settings
- Line3 `input_file` : rosbag path used for simulation.
- Line4 `output_file` : Output destination for simulation results.
- Line5 `spoofing_mode` : type of spoofing, if "removal", HFR attack, if "static" inject false wall.
- Line6, 7 `spoofer_x` and `spoofer_y` : Spoofer placement position. (specified by x and y coordinates)
- Line8 `distance_threshold` : Distance at which Spoofer can track and attack. Spoofing occurs when the target enters within a radius of distance_threshold meters from the attacking device.
- Line9 `lidar_topic` : Input rosbag pointcloud topic name.
  - Line10 `lidar_topic_length` : If you use Velodyne LiDAR, set value 22. If you use Livox LiDAR, set value 18.
- Line12 `spoofed_topic` : Topic name of the simulated PointCloud. It is recommended not to change from the original Topic name.
- Line13 `spoofing_range` : The horizontal azimuthal range of the pointcloud that Spoofer can tamper with in a single frame.
- Line14 `wall_dist` : Used only in static mode. How far away from the target the wall is injected. (Unit : meter)
- Line17 `horizontal_resolution` : Horizontal resolution of LiDAR.
- Line18 `vertical_lines`: Number of LiDAR vertical scan lines.
- Line19 ` spoofing_rate` : Values between 0.0 and 1.0. The higher the value, the more random noise is injected during the HFR attack. Default values are experimental and recommended ones.
- Line22 `minimum_measureing_distance` : The simulation truncates the pointcloud below a certain distance from LiDAR during simulation.
- Line23 `maximum_measureing_distance` : The simulation truncates the pointcloud beyond a certain distance from LiDAR during the simulation.
- Line24 `minimum_height_htreshold` : Eliminate points below a certain height during simulation to remove the influence of the ground pointcloud.
