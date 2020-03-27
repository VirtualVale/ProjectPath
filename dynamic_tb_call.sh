#!/bin/bash
trap "kill $(jobs -p); exit 187;" SIGTERM
x_pos=(0 0 0 0 0 0 0 0 0 0)
y_pos=(0 0 1 2 3 4 5 6 7 8)
number=$(zenity --entry --text "How many robots should spawn in the simulation?
(Valid are integers between 1 and 8)" --entry-text "1" --title "Number of robots" 2> /dev/null) 
echo "$number"
if [ $number -le 0 -o $number -ge 9 ]
    then
        echo "Wrong input!"
        exit  255
fi
./dynamic_rviz_config.sh $number
roslaunch path_planning_system dynamic_gazebo_map_rviz.launch &
sleep 5s
if ! zenity --warning --text "$number robots are spawned!" 2> /dev/null; then
  kill $(jobs -p)
  exit 255;
fi
counter=1
while [ $counter -le $number ]
do
    roslaunch path_planning_system dynamic_spawn.launch tb_number:=$counter y_pos:=${y_pos[$counter]} &
    sleep 5s
    ((counter++))
done
sleep 5s
echo press k to shutdown or Strg+C
kill_command="o"
while [ $kill_command != "k" ]
do
    read kill_command
done
kill $(jobs -p)
#roslaunch path_planning_system setup.launch
