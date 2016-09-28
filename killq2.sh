#!/bin/bash

rosparam set /q2/pathfollowing/PF_mode 0
rosservice call /q2/update_params [pathfollowing/PF_mode]

