# Experiments Radar-Aruco to avoid Smoke

This repo contains some configuration files to use in the research of smoke avoidance navigation and localization.
To download bag files to use with these launch files go to the dropbox folder:

[DropBox Bag Folder](https://www.dropbox.com/sh/kt2q8ulnx4171dk/AACkiA1Xx5g9ah2zWdHbek7Fa?dl=0)

[DropBox MultiRadar Bag Folder](https://www.dropbox.com/sh/k868avekgrstwzs/AADj7J1sI7gRccPbI_15fMdOa?dl=0)

By **default** the launch ```make_aruco_map.launch``` is configured to avoid using the /tf and /tf_static topics in the case they are present on the bag file. So if you need to use these topics because some reasons, just run the launch file with the arg *use_bag_tf* set to true.

Of course, to use this launch you will need to download some ROS Packages such as [MCL3D](https://github.com/robotics-upo/mcl3d/) [Fiducials](https://github.com/robotics-upo/fiducials) and [Odom To Tf](https://github.com/robotics-upo/odom_to_tf). [Cloud Concatenator](https://github.com/robotics-upo/cloud_concatenator). Just follow the instructions on every package, clone them into your catkin workspace an compile.

## Localize your robot

The launch file come with a default position in the attached map, but it happens that some bags are recorded from a different start position, so you will need to initially localize them, using "P" key in the RViz window.

## Sensor Fusion

### Dependencies
    
- Cloud concatenator: https://github.com/robotics-upo/cloud_concatenator
- pcl_ros: ```sudo apt install ros-$ROS_DISTRO-pcl-ros```
- pointcloud_to_laser_scan ```sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan```
- raposa_marker: 
- odom_to_tf

You can downlaod the repository and execute ```./scripts/install_fusion_dependencies.sh``` it will install APT packages and downlaod necessary repos to avoid doing it manually. 

To test the radar and lidar fusion node you will need a dataset. You can download it [here](https://www.dropbox.com/s/qr5nr5gbng1jflh/2020-12-01-13-21-03.bag?dl=0) a bag containing pointcloud and radar data from December experiments. Download into your home folder and launch the main launch as it follows:

```
roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar.txt localize_method:=fusion
```

It will run the fusion node and launch a rviz visualization window where you can select which pointclouds you want to visualize.

The implementation steps are detailed in the document docs/Lidar_and_radar_fusion_implementation_details.pdf 

### Parameters

Related to the algorithms:

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
