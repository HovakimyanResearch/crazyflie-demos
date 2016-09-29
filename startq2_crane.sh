#!/bin/bash

source devel/setup.bash

echo 'Initialize Vicon position'
rosservice call /q2/initialize_vicon_pos > /dev/null

echo 'Turning on LEDs'
rosparam set /q2/ring/headlightEnable 1
rosservice call /q2/update_params [ring/headlightEnable] > /dev/null
rosparam set /q2/ring/effect 7
rosservice call /q2/update_params [ring/effect] > /dev/null

echo 'Enable pathfollowing mode'
rosparam set /q2/pathfollowing/PF_mode 1
rosservice call /q2/update_params [pathfollowing/PF_mode] > /dev/null

echo 'Run "Crane.py"'
rosrun crazyflie_demo Crane.py

echo 'Killing Percy'
./killq2.sh > /dev/null

echo 'Turning off LEDs'
rosparam set /q2/ring/headlightEnable 0
rosservice call /q2/update_params [ring/headlightEnable] > /dev/null
rosparam set /q2/ring/effect 0
rosservice call /q2/update_params [ring/effect] > /dev/null

