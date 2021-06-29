#! /bin/bash
CONTADOR=$1
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  echo Launch experiment $CONTADOR
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_27_nov_12_14_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_14_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_14_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_27_nov_12_24_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/2020-11-27-12-24-27_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_24_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-12-24-27_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_24_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-12-24-27_gt_refined.bag
  # low density
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_27_nov_12_41_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/low_density/2020-11-27-12-41-51_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_41_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-12-41-51_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_41_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-12-41-51_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_27_nov_12_47_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/2020-11-27-12-47-13_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_47_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-12-47-13_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_47_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-12-47-13_gt_refined.bag
  #mid density
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_27_nov_12_57_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/low_density/2020-11-27-12-57-54_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_57_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-12-57-54_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_57_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-12-57-54_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_27_nov_13_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/2020-11-27-13-09-07_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_13_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-13-09-07_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_13_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-13-09-07_gt_refined.bag
  
  let CONTADOR+=1
done
