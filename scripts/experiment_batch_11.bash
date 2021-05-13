#! /bin/bash
CONTADOR=$1
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  echo Launch experiment $CONTADOR
  export X=6.3
  export Y=4.6
  export A=1.6

  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_fusion_11_$CONTADOR.txt localize_method:=fusion bag_name:=2020-12-01-11-36-21_odom_gt.bag initial_pose_x=$X initial_pose_y=$X initial_pose_a=$A
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_11_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=2020-12-01-11-36-21_odom_gt.bag initial_pose_x=$X initial_pose_y=$X initial_pose_a=$A
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_11_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=2020-12-01-11-36-21_odom_gt.bag initial_pose_x=$X initial_pose_y=$X initial_pose_a=$A
  let CONTADOR+=1
done