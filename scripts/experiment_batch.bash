#! /bin/bash
CONTADOR=$1
export X=6.25
export Y=4.5
export A=1.55
export BAG=2020-12-01-12-04-38_gt_fusion.bag
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  echo Launch experiment $CONTADOR
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_12_$CONTADOR.txt localize_method:=fusion initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A bag_name:=$BAG scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_12_$CONTADOR.txt localize_method:=fusion initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A bag_name:=$BAG
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_12_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A bag_name:=$BAG
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_12_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A bag_name:=$BAG
  roslaunch radar_experiments localize_with_gt_fritsche.launch gt_filename:=stats_fritsche2_12_$CONTADOR.txt bag_name:=$BAG initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A  range_filename:=radar_stats_fritsche_12.m
  let CONTADOR+=1
done