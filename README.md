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

Go to the /devel folder and run the following command
```
source setup.bash
```

Run the following commands in separate terminals in order to run the crane demo:
```
roscore
rosrun joy joy_node
roslaunch crazyflie_driver crazyflie_percy.launch
./startcf_crane.sh
```
