


if [ $# -lt 2 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number> [<output_stats_folder>]"
  exit 1
fi

CONTADOR=$1

if [ $# -gt 2 ]; then
  echo "Saving stats to: $3"
fi

X=6.2
Y=11.55
A=-1.52
BAG=$HOME/Humo/corchopan/2020-11-11-14-34-17.bag

# roslaunch radar_experiments localize_gt.launch bag_file:=$BAG stats_file:=stats_corcho_gt_1.txt map_name:=map_corchopan_v1.txt

until [ $CONTADOR -gt $2 ]; do
  roslaunch radar_experiments localize_lidar.launch initial_x:=$X initial_y:=$Y initial_a:=$A bag_file:=$BAG stats_file:=stats_corcho_lidar_1_$CONTADOR.txt map_name:=corchopan_v2.bt
  roslaunch radar_experiments localize_radar.launch initial_x:=$X initial_y:=$Y initial_a:=$A bag_file:=$BAG stats_file:=stats_corcho_radar_1_$CONTADOR.txt map_name:=corchopan_v2.bt
  let CONTADOR+=1
done
