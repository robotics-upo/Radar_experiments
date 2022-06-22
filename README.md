# Localization experiments. 

This repository contains the source code and additional configuration files to reproduce the results of our work submitted to Robot 2022:

D. Alejo, R. Rey, J. A. Cobano, F. Caballero and L. Merino, "Data  fusion  of  low-cost  RADAR  and  LIDAR  for  reliable  ground  robotlocalization  under  low-visibility  conditions"

In this paper, we have designed and implemented a new method for fusing radar and lidar information. The method has been implemented mainly in the source file: src/sensor_fusion/lidar_radar_fusion_dbscan.cpp

The dataset used in this paper can be downloaded at:

[Dataset web page](https://robotics.upo.es/datasets/smoke)

## Dependencies

The utilities of this repository have been programmed on a Linux 18.04 distribution and ROS melodic. They have also been tested in Ubuntu 20.04 and ROS noetic. Please refer to the [ROS wiki](https://wiki.ros.org/ROS/Installation) for details on installation and the creation of a catkin workspace.
    
- Cloud concatenator: https://github.com/robotics-upo/cloud_concatenator
- pcl_ros: ```sudo apt install ros-$ROS_DISTRO-pcl-ros```
- pointcloud_to_laser_scan ```sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan```
- raposa_marker: https://github.com/robotics-upo/upo_markers
- odom_to_tf:  https://github.com/robotics-upo/odom_to_tf

You can download the repository and execute ```./scripts/install_fusion_dependencies.sh``` from the source directory of a catkin workspace. It will install APT packages and download the necessary repos to avoid doing it manually. In addition it asks you if you want to download the dataset.

Once you download our dataset into the folder "${HOME}/radar_fusion_bags", you can launch the experiment batch presented in the ICRA paper as follows (for ten repetitions):

```
rosrun radar_experiments experiment_batch.sh 1 10
```

It will run the fusion node and launch a rviz visualization window where you can select which radar scans you want to visualize.

### Parameters for the implementation of P. Fritsche & B. Wagner 2017 "Modeling structure and aerosol concentration with fused radar and LiDAR data in environments with changing visibility" (IROS 2017)

Related to the algorithm:

- yaw_tolerance : Used to extract 2d virtual scans
- ransac_distance_threshold (for both lidar and radar)
- ransac_iterations (for both lidar and radar)
- min_ransac_pointcloud_size (for both lidar and radar)
- beta
- distance_threshold
- lidar_dev
- radar_dev
- lidar_max_range

Related to node:

- sychronization_margin
- time_tolerance
