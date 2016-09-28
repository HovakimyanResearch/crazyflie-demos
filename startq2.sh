#!/bin/bash
rosservice call /q2/initialize_vicon_pos
read -n 1 -s
rosparam set /q2/pathfollowing/PF_mode 1
read -n 1 -s
rosservice call /q2/update_params [pathfollowing/PF_mode]
rosrun crazyflie_demo test1_q2.py
