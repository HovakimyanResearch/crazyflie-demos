#!/bin/bash

rosservice call /danny/initialize_vicon_pos
read -n 1 -s

rosparam set /danny/ring/effect 2
rosservice call /danny/update_params [ring/effect]
read -n 1 -s

rosparam set /danny/pathfollowing/PF_mode 1
read -n 1 -s

rosservice call /danny/update_params [pathfollowing/PF_mode]
rosrun crazyflie_demo path_from_csv_vertical_yz_danny.py
