# Localization experiments. Submitted to ICRA 2022

This repository contains the source code and additional configuration files to reproduce the results of our work submitted to ICRA2022:

D. Alejo, R. Rey, J. A. Cobano, F. Caballero and L. Merino, "Data  fusion  of  low-cost  RADAR  and  LIDAR  for  reliable  ground  robotlocalization  under  low-visibility  conditions"

The dataset used in this paper can be downloaded at:

[Dataset web page](https://robotics.upo.es/datasets/smoke)

[DropBox MultiRadar Bag Folder](https://www.dropbox.com/sh/k868avekgrstwzs/AADj7J1sI7gRccPbI_15fMdOa?dl=0)

By **default** the launch ```make_aruco_map.launch``` is configured to avoid using the /tf and /tf_static topics in the case they are present on the bag file. So if you need to use these topics because some reasons, just run the launch file with the arg *use_bag_tf* set to true.

## Dependencies
    
- Cloud concatenator: https://github.com/robotics-upo/cloud_concatenator
- pcl_ros: ```sudo apt install ros-$ROS_DISTRO-pcl-ros```
- pointcloud_to_laser_scan ```sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan```
- raposa_marker: https://github.com/robotics-upo/upo_markers
- odom_to_tf:  https://github.com/robotics-upo/odom_to_tf

You can downlaod the repository and execute ```./scripts/install_fusion_dependencies.sh``` from the source directory of a catkin workspace. It will install APT packages and downlaod necessary repos to avoid doing it manually. In addition it asks you if you want to download the dataset.

Once you download our dataset into your home folder, you can launch the main an experiment as follows:

```
roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar.txt localize_method:=fusion
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
