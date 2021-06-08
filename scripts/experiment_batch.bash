#! /bin/bash
CONTADOR=$1
export X=6.25
export Y=4.5
export A=1.55
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  echo Launch experiment $CONTADOR
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_fusion_12_$CONTADOR.txt localize_method:=fusion initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A 
  # roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_12_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A 
  # roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_12_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan initial_pose_x:=$X initial_pose_y:=$Y initial_pose_a:=$A 
  let CONTADOR+=1
done