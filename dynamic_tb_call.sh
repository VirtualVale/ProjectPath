#!/bin/bash
#this script starts a simulation in Gazebo with a dynamic number of turtlebots
#trap catches SIGTERM aka Strg+C and kills all processes
trap "kill $(jobs -p); exit 187;" SIGTERM
#arrays to define the spawn locations for the turtlebots (first column is not used)
x_pos=(0 0 0 0 0 0 0 0 0 0)
y_pos=(0 0 1 2 3 4 5 6 7 8)
#number turtlebots
number=$(zenity --entry --text "How many robots should spawn in the simulation?
(Valid are integers between 1 and 8)" --entry-text "1" --title "Number of robots" 2> /dev/null) 
echo "$number"
if [ $number -le 0 -o $number -ge 9 ]
    then
        echo "Wrong input!"
        exit  255
fi
#this scripts writes a rviz config file
./dynamic_rviz_config.sh $number
roslaunch path_planning_system dynamic_gazebo_map_rviz.launch &
sleep 5s
#warning gives the possibility to stop the process
if ! zenity --warning --text "$number robots are spawned!" 2> /dev/null; then
  kill $(jobs -p)
  exit 255;
fi
#spawn of turtlebots
counter=1
while [ $counter -le $number ]
do
    roslaunch path_planning_system dynamic_spawn.launch tb_number:=$counter y_pos:=${y_pos[$counter]} &
    sleep 5s
    ((counter++))
done
sleep 5s
#termination process
echo press k to shutdown or Strg+C
kill_command="o"
while [ $kill_command != "k" ]
do
    read kill_command
done
kill $(jobs -p)
