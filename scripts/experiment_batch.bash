#! /bin/bash
if [[ $# -ne 2 ]]
then
  echo "Usage: $0 <min_iteration> <max_iteration>"
fi

CONTADOR=$1

until [ "$CONTADOR" -gt "$2" ]; do

  # Roslaunch with multiple parameters

  echo Launch repetition $CONTADOR
  for ((i=1;i<=7;i++))
  do
    echo Launch experiment $i
    roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_clustered_experiment_${i}_$CONTADOR.txt localize_method:=fusion  bag_name:="experiment_${i}.bag" scan_topic:=/dbscan_fuser/segmented_scan points_stats_filename:="points_experiment_${i}.m"
    roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_dbscan_lines_experiment_${i}_$CONTADOR.txt localize_method:=fusion bag_name:="experiment_${i}.bag" points_stats_filename:="points_experiment_${i}.m"
    roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_radar_experiment_${i}_$CONTADOR.txt localize_method:=radar scan_topic:=radar_scan bag_name:="experiment_${i}.bag" points_stats_filename:="points_experiment_${i}.m"
    roslaunch radar_experiments localize_with_gt.launch gt_filename:=stats_lidar_experiment_${i}_$CONTADOR.txt localize_method:=lidar scan_topic:=lidar_scan bag_name:="experiment_${i}.bag" points_stats_filename:="points_experiment_${i}.m"
    roslaunch radar_experiments localize_with_gt_fritsche.launch gt_filename:=stats_fritsche2_experiment_${i}_$CONTADOR.txt bag_name:="experiment_${i}.bag" range_filename:="radar_stats_fritsche_experiment_${i}.m"
  done
  (( CONTADOR+=1 )) 
done
