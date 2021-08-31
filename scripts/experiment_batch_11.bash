#! /bin/bash
CONTADOR=$1
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  echo Launch experiment $CONTADOR
  export X=6.1
  export Y=4.4
  export A=1.4
  export BAG=2020-12-01-11-36-21_gt_fusion.bag

  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_11_$CONTADOR.txt localize_method:=segmented bag_name:=$BAG initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_11_$CONTADOR.txt localize_method:=fusion bag_name:=$BAG initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_11_$CONTADOR.txt localize_method:=lidar bag_name:=2020-12-01-11-36-21_refined.bag initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A scan_topic:=/lidar_scan 
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_11_$CONTADOR.txt localize_method:=radar bag_name:=2020-12-01-11-36-21_refined.bag initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A scan_topic:=/radar_scan 

  let CONTADOR+=1
done