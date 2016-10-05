#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Please provide drone name as the first argument"
    exit
fi

source devel/setup.bash

echo "Killing \"$1\""
rosparam set /"$1"/pathfollowing/PF_mode 0
rosservice call /"$1"/update_params [pathfollowing/PF_mode] > /dev/null
