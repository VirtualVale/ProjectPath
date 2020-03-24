#!/bin/bash
x_pos=(0 0 0 0 0 0 0 0 0 0)
y_pos=(0 0 1 2 3 4 5 6 7 8)
number=$(zenity --entry --text "How many robots should spawn in the simulation?
(Valid are integers between 1 and 10)" --entry-text "1" --title "Number of robots" 2> /dev/null) 
echo "$number"
roslaunch path_planning_system dynamic_gazebo_map_rviz.launch &
if [ $number -le 0 -o $number -ge 10 ]
    then
        echo "Wrong input!"
        exit  255
fi
zenity --info --text "Valo spawns $number robots!" 2> /dev/null
counter=1
while [ $counter -le $number ]
do
    echo $counter
    echo ${y_pos[$counter]}
    roslaunch path_planning_system single_tb_call.launch tb_number:=$counter y_pos:=${y_pos[$counter]} &
    ((counter++))
done
echo All done
echo press k to shutdown
kill_command="o"
while [ $kill_command != "k" ]
do
    read kill_command
done
echo kill
kill $(jobs -p)
#roslaunch path_planning_system setup.launch
