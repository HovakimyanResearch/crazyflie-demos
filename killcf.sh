#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Please provide drone names as arguments"
    exit
fi

source devel/setup.bash

for i in "$@"; do
    echo "Killing \"$i\""
    rosparam set /"$i"/pathfollowing/PF_mode 0
    rosservice call /"$i"/update_params [pathfollowing/PF_mode] > /dev/null
done
