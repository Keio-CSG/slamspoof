# Slamspoof

# What is Slamspoof ? 
Slamspoof is a framework that analyzes LiDAR SLAM and reveals vulnerabilities to adversarial laser pulse injections, or LiDAR spoofing. The vulnerabilities are quantitatively evaluated using the SMVS (Scan Matching Vulnerability Score) metric, and the values can be referenced across the entire map. Please note that this repository focuses on localization using LiDAR-based localization (Visual Odometry is not covered, sorry). Additionally, the framework supports kiss-icp and A-LOAM as localization methods.

# Related repositories
- [evo](https://github.com/MichaelGrupp/evo)
  - The output results are generated in a format that can be evaluated using evo. 
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
- A-LOAM
  - Install [here](https://github.com/HKUST-Aerial-Robotics/A-LOAM)
- small_gicp
  - Install [here](https://github.com/koide3/small_gicp). We recommend installation from source.

- Python packages
  - numpy
  - pandas
  - matplotlib
  - open3d
    - python 3.6-3.9 only
      
# Install Slamspoof
First, confirm prerequisites are correctly installed.
```
# for ROS noetic
cd your_ros_workspace/src
git clone https://hogehoge.git
cd .. && catkin build 
source devel/setup.bash
cd your_ros_workspace/src
./initial_setup.sh
```

# How to use
After installing prerequisites, clone this repo.
```
cd your_ros_workspace/src
https://github.com/ngtroku/slamspoof.git
cd .. && catkin build
source devel/setup.bash
```
## Create no-attack trajectory
- Line 4: The parameter `no_attack_trajectory` specifies the output destination for the trajectory.
- Line 8: The `bagfile` specifies the rosbag file to be played, but you can leave it empty and use the rosbag play command to play the rosbag file.
- Line 12: The `topic` is the rostopic of the point cloud used for trajectory calculation. Please use the sensor_msgs/PointCloud2 type for the rostopic.

  ```
  roslaunch slamspoof benign_map.launch
  ```
  Launch other terminal and run below command.
  ```
  rosbag play (your_rosbag).bag
  ```
## Vulnerablity analysis & spoofing simulation
This Slamspoof repository supports vulnerablity analysis and spoofing simulation. 
### Simulation on kiss-icp
```
roslaunch slamspoof run_simulation_kiss.launch
```
and launch other terminal to play rosbag.
```
rosbag play (your_rosbag).bag
```
- Line 4: The parameter `subscribe_topic_name` is the point cloud topic name for localization. Topic must be set sensor_msgs/PointCloud2.
- Line 13: The `spoofing mode` allows you to choose 'removal' or 'injection.' In removal mode, the point cloud within the specified range is erased. In injection mode, fake walls are injected within the specified range.
- Line 14: The `injection_distance` is used only when the spoofing mode is set to injection. It is a parameter that determines how far from the LiDAR the fake walls will be injected (unit: m).
- Line 15: The `reference_trajectory_file` is the path to the no-attack trajectory created with "Create no-attack trajectory" chapter.
- Line 21, 22: Arguments `x` and `y` determine the coordinates for placing the spoofer during the spoofing simulation.
- Line 24: Spoofing occurs when the distance between the spoofer's installation position and the target LiDAR is less than or equal to the set value of `spoofing_max_distance`. Setting this value to 0 will disable spoofing."
- Line 31: `lidar_topic_length` is the value of point_step for the point cloud topic set in line 4. For example, the point_step value for VLP-16’s /velodyne_points is 22.
### Simulation on A-LOAM
```
roslaunch slamspoof run_simulation_loam.launch
```
and launch other terminal to play rosbag.
```
rosbag play (your_rosbag).bag
```
## Visualize vulnerablity
Based on the results obtained from the vulnerability analysis, we will seek candidates for the objects that localization program depends on.
Before run below command, set `vulnerablity_file_path` in visualizer.launch. Example : `<param name="vulnerablity_file_path" value="$(find slamspoof)/Vulnerablity/your_analysis_result.csv"/>`
```
roslaunch slamspoof visualizer.launch
```
