#! /bin/bash
CONTADOR=$1

export BAG7=exp27nov/2020-11-27-12-24-27_gt_refined.bag
export BAG6=exp27nov/low_density/2020-11-27-12-41-51_gt_refined.bag
export BAG5=exp27nov/low_density/2020-11-27-12-47-13_gt_refined.bag
export BAG4=exp27nov/mid_density/2020-11-27-12-57-54_gt_refined.bag
export BAG3=exp27nov/mid_density/2020-11-27-13-09-07_gt_refined.bag

until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  echo Launch experiment $CONTADOR
  # roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_27_nov_12_14_$CONTADOR.txt localize_method:=segmented bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag scan_topic:=/dbscan_fuser/segmented_scan
  # roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_27_nov_12_14_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag
  # roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_14_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag
  # roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_14_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag


  #Experiment 7
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_exp7_$CONTADOR.txt localize_method:=segmented bag_name:=$BAG7 scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_exp7_$CONTADOR.txt localize_method:=fusion bag_name:=$BAG7
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_exp7_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=$BAG7
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_exp7_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=$BAG7
  # low density
  #Experiment 6
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_exp6_$CONTADOR.txt localize_method:=segmented bag_name:=$BAG6 scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_exp6_$CONTADOR.txt localize_method:=fusion bag_name:=$BAG6
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_exp6_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=$BAG6
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_exp6_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=$BAG6
  #Experiment 5
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_exp5_$CONTADOR.txt localize_method:=fusion bag_name:=$BAG5 scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_exp5_$CONTADOR.txt localize_method:=fusion bag_name:=$BAG5
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_exp5_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=$BAG5
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_exp5_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=$BAG5
  #mid density
  #Experiment 4
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_exp4_$CONTADOR.txt localize_method:=segmented bag_name:=$BAG4 scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_exp4_$CONTADOR.txt localize_method:=fusion bag_name:=$BAG4
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_exp4_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=$BAG4
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_exp4_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=$BAG4
  #Experiment 3
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_exp3_$CONTADOR.txt localize_method:=segmented bag_name:=$BAG3 scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_exp3_$CONTADOR.txt localize_method:=fusion bag_name:=$BAG3
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_exp3_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=$BAG3
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_exp3_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=$BAG3

  #Fritsche 
  # Exp 7
  roslaunch radar_experiments localize_with_gt_fritsche.launch gt_filename:=stats_fritsche_exp7_$CONTADOR.txt  bag_name:=$BAG7  range_filename:=radar_stats_fritsche_exp7.m
  # Exp 6
  roslaunch radar_experiments localize_with_gt_fritsche.launch gt_filename:=stats_fritsche_exp6_$CONTADOR.txt  bag_name:=$BAG6 range_filename:=radar_stats_fritsche_exp6.m
  # Exp 5
  roslaunch radar_experiments localize_with_gt_fritsche.launch gt_filename:=stats_fritsche_exp5_$CONTADOR.txt  bag_name:=$BAG5 range_filename:=radar_stats_fritsche_exp5.m
  # Exp 4
  roslaunch radar_experiments localize_with_gt_fritsche.launch gt_filename:=stats_fritsche_exp4_$CONTADOR.txt  bag_name:=$BAG4 range_filename:=radar_stats_fritsche_exp4.m
  # Exp 3
  roslaunch radar_experiments localize_with_gt_fritsche.launch gt_filename:=stats_fritsche_exp3_$CONTADOR.txt  bag_name:=$BAG3 range_filename:=radar_stats_fritsche_exp3.m
  let CONTADOR+=1
done
