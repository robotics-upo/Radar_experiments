


if [ $# -ne 4 ]; then
  echo "Usage: $0 <first_experiment> <last_experiment> <first_exe_number> <last_exe_number> "
  exit 1
fi

CONTADOR=$1




until [ $CONTADOR -gt $2 ]; do
  rosrun radar_experiments experiment_$CONTADOR.bash $3 $4
  let CONTADOR+=1
done


