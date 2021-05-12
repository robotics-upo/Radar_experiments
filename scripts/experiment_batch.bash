#! /bin/bash
CONTADOR=$1
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  echo Launch experiment $CONTADOR
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_fusion_$CONTADOR.txt localize_method:=fusion
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_$CONTADOR.txt localize_method:=radar
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_$CONTADOR.txt localize_method:=lidar
  let CONTADOR+=1
done