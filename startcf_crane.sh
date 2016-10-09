#!/bin/bash

source devel/setup.bash

echo 'Initialize Vicon position'
rosservice call /percy/initialize_vicon_pos > /dev/null

echo 'Turning on LEDs'
rosparam set /percy/ring/headlightEnable 1
rosservice call /percy/update_params [ring/headlightEnable] > /dev/null
rosparam set /percy/ring/effect 7
rosservice call /percy/update_params [ring/effect] > /dev/null

echo 'Run "Crane.py"'
rosrun crazyflie_demo Crane.py

# echo 'Enable pathfollowing mode'
# rosparam set /percy/pathfollowing/PF_mode 1
# rosservice call /percy/update_params [pathfollowing/PF_mode] > /dev/null

echo 'Killing Percy'
./killcf.sh percy > /dev/null

echo 'Turning off LEDs'
rosparam set /percy/ring/headlightEnable 0
rosservice call /percy/update_params [ring/headlightEnable] > /dev/null
rosparam set /percy/ring/effect 0
rosservice call /percy/update_params [ring/effect] > /dev/null

