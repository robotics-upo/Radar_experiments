


if [ $# -lt 2 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number> [<output_stats_folder>]"
  exit 1
fi

CONTADOR=$1

if [ $# -gt 2 ]; then
  echo "Saving stats to: $3"
fi

X=5.9
Y=10.8
A=-1.59

# roslaunch radar_experiments localize_gt.launch initial_x:=5.9 initial_y:=10.46 initial_a:=-1.59 bag_file:=$HOME/Humo/miercoles_28oct/2020-10-28-11-26-44_0.bag stats_file:=stats_gt_0.txt

until [ $CONTADOR -gt $2 ]; do
  roslaunch radar_experiments localize_lidar.launch initial_x:=$X initial_y:=$Y initial_a:=$A bag_file:=$HOME/Humo/miercoles_28oct/2020-10-28-11-26-44_0.bag stats_file:=stats_lidar_0_$CONTADOR.txt
  roslaunch radar_experiments localize_radar.launch initial_x:=$X initial_y:=$Y initial_a:=$A bag_file:=$HOME/Humo/miercoles_28oct/2020-10-28-11-26-44_0.bag stats_file:=stats_radar_0_$CONTADOR.txt
  let CONTADOR+=1
done


