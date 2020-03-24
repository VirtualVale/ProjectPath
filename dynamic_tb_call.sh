#!/bin/bash
number=$(zenity --entry --text "how many robots" --entry-text "1" --title "number robots" 2> /dev/null) 
echo "$number"
zenity --info --text "Valo spawns $number robots!" 2> /dev/null
counter=1
while [ $counter -le $number ]
do
    echo $counter
    ((counter++))
done

echo All done
#roslaunch path_planning_system setup.launch
