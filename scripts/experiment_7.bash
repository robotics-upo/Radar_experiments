


if [ $# -lt 2 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number> [<output_stats_folder>]"
  exit 1
fi

CONTADOR=$1

if [ $# -gt 2 ]; then
  echo "Saving stats to: $3"
fi

X=6
Y=12.05
A=-1.52
BAG=$HOME/Humo/miercoles_28oct/2020-10-28-12-35-57_7.bag

roslaunch radar_experiments localize_gt.launch bag_file:=$BAG stats_file:=stats_gt_7.txt

until [ $CONTADOR -gt $2 ]; do
  roslaunch radar_experiments localize_lidar.launch initial_x:=$X initial_y:=$Y initial_a:=$A bag_file:=$BAG stats_file:=stats_lidar_7_$CONTADOR.txt odom_x_mod:=0.1 odom_x_mod:=0.1 odom_a_mod:=1.0
  roslaunch radar_experiments localize_radar.launch initial_x:=$X initial_y:=$Y initial_a:=$A bag_file:=$BAG stats_file:=stats_radar_7_$CONTADOR.txt odom_x_mod:=0.1 odom_x_mod:=0.1 odom_a_mod:=1.0
  let CONTADOR+=1
done


