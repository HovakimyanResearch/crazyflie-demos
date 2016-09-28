#!/bin/bash
rosservice call /percy/initialize_vicon_pos
rosservice call /danny/initialize_vicon_pos
read -n 1 -s
rosparam set /percy/ring/effect 2
rosparam set /danny/ring/effect 2
rosservice call /percy/update_params [ring/effect]
rosservice call /danny/update_params [ring/effect]
read -n 1 -s
rosparam set /percy/pathfollowing/PF_mode 1
rosparam set /danny/pathfollowing/PF_mode 1
read -n 1 -s
rosservice call /percy/update_params [pathfollowing/PF_mode]
rosservice call /danny/update_params [pathfollowing/PF_mode]
rosrun crazyflie_demo traj2d_orbit.py
