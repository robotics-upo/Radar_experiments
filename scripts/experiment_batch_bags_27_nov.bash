#! /bin/bash
CONTADOR=$1
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  echo Launch experiment $CONTADOR
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_27_nov_12_14_$CONTADOR.txt localize_method:=segmented bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_27_nov_12_14_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_14_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_14_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-12-14-42_gt_refined.bag


  #Experiment 7
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_27_nov_12_24_$CONTADOR.txt localize_method:=segmented bag_name:=exp27nov/2020-11-27-12-24-27_gt_refined.bag scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_27_nov_12_24_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/2020-11-27-12-24-27_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_24_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/2020-11-27-12-24-27_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_24_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/2020-11-27-12-24-27_gt_refined.bag
  # low density
  #Experiment 6
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_27_nov_12_41_$CONTADOR.txt localize_method:=segmented bag_name:=exp27nov/low_density/2020-11-27-12-41-51_gt_refined.bag scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_27_nov_12_41_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/low_density/2020-11-27-12-41-51_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_41_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/low_density/2020-11-27-12-41-51_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_41_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/low_density/2020-11-27-12-41-51_gt_refined.bag
  #Experiment 5
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_27_nov_12_47_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/low_density/2020-11-27-12-47-13_gt_refined.bag scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_27_nov_12_47_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/low_density/2020-11-27-12-47-13_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_47_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/low_density/2020-11-27-12-47-13_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_47_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/low_density/2020-11-27-12-47-13_gt_refined.bag
  #mid density
  #Experiment 4
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_27_nov_12_57_$CONTADOR.txt localize_method:=segmented bag_name:=exp27nov/mid_density/2020-11-27-12-57-54_gt_refined.bag scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_27_nov_12_57_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/mid_density/2020-11-27-12-57-54_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_12_57_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/mid_density/2020-11-27-12-57-54_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_12_57_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/mid_density/2020-11-27-12-57-54_gt_refined.bag
  #Experiment 3
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_segmented_27_nov_13_$CONTADOR.txt localize_method:=segmented bag_name:=exp27nov/mid_density/2020-11-27-13-09-07_gt_refined.bag scan_topic:=/dbscan_fuser/segmented_scan
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_27_nov_13_$CONTADOR.txt localize_method:=fusion bag_name:=exp27nov/mid_density/2020-11-27-13-09-07_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_27_nov_13_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:=exp27nov/mid_density/2020-11-27-13-09-07_gt_refined.bag
  roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_27_nov_13_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:=exp27nov/mid_density/2020-11-27-13-09-07_gt_refined.bag
  
  let CONTADOR+=1
done
