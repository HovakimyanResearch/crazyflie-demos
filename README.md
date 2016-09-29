# Code for the Pygmalion Festival

First download and install ROS Indigo: <http://wiki.ros.org/indigo/Installation/Ubuntu>.

Then build this ROS packages:
```
catkin_make
```

### DroneDraw

Run the following commands in two separate terminals in order run the drone draw demo:
```
roslaunch crazyflie_driver crazyflie_percy.launch
./startcf_percy_vertical_xz.sh
```
The source for the Android app is located here: <https://github.com/Lauszus/DroneDraw>.

### Crane demo
(1) Select correct drone. Go to < directory where you cloned this git >/src/crazyflie_ros/crazyflie_demo/scripts and edit the file Crane.py. In the main function you must choose which version of veh1. So far we have either percy or q2. One option will be enabled and the other will be commented.

(2) Go to the devel folder and run the following command
```
source setup.bash
```

Run the following commands in separate terminals in order to run the crane demo and replace DRONENAME by either q2 or percy:
```
roscore
rosrun joy joy_node
roslaunch crazyflie_driver crazyflie_DRONENAME.launch
./startcf_crane.sh
```

Do not forget to set the correct Vicon IP at src/vicon_bridge/launch/vicon.launch

