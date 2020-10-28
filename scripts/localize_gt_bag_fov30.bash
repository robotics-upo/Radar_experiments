


if [ $# -lt 2 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number> [<output_stats_folder>]"
  exit 1
fi

CONTADOR=$1

if [ $# -gt 2 ]; then
  echo "Saving stats to: $3"
fi

until [ $CONTADOR -gt $2 ]; do
  roslaunch localize_gt.launch initial_x:=5.9 initial_y:=10.46 initial_a:=-1.59 bag_name:=multi_radarfov30_corrected.bag stats_file:=stats_gt_fov30_$CONTADOR.txt
  let CONTADOR+=1
done
