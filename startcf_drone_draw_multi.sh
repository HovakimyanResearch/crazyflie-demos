#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Please provide drone names as arguments"
    exit
fi

echo "Using "$#" drones: $@"

source devel/setup.bash

for i in "$@"; do
    echo "Initialize Vicon position for \"$i\""
    rosservice call /"$i"/initialize_vicon_pos > /dev/null
done

echo 'Run "drone_draw_multi.py"'
rosrun crazyflie_demo drone_draw_multi.py "$@"

./killcf.sh "$@"
