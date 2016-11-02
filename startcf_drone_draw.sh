#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Please provide drone name as the first argument"
    exit
fi

echo "Using \"$1\" as the drone"

source devel/setup.bash

echo 'Initialize Vicon position'
rosservice call /"$1"/initialize_vicon_pos > /dev/null

echo 'Enable pathfollowing mode'
rosparam set /"$1"/pathfollowing/PF_mode 1
rosservice call /"$1"/update_params [pathfollowing/PF_mode] > /dev/null

echo 'Run "drone_draw.py"'
rosrun crazyflie_demo drone_draw.py "$1"

./killcf.sh "$1"
