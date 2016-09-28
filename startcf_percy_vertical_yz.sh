#!/bin/bash

rosservice call /percy/initialize_vicon_pos
read -n 1 -s

rosparam set /percy/ring/effect 2
rosservice call /percy/update_params [ring/effect]
read -n 1 -s

rosparam set /percy/pathfollowing/PF_mode 1
read -n 1 -s

rosservice call /percy/update_params [pathfollowing/PF_mode]
rosrun crazyflie_demo path_from_csv_vertical_yz_percy.py
