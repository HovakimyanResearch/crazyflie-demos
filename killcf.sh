#!/bin/bash

rosparam set /percy/pathfollowing/PF_mode 0
rosservice call /percy/update_params [pathfollowing/PF_mode]
rosparam set /danny/pathfollowing/PF_mode 0
rosservice call /danny/update_params [pathfollowing/PF_mode]
