# Code for the Pygmalion Festival

First download and install ROS Indigo: <http://wiki.ros.org/indigo/Installation/Ubuntu>.

In order to build the ROS packages:
```
catkin_make
```

Run the following commands in two separate terminals in order run the drone draw demo:
```
roslaunch crazyflie_driver crazyflie_percy.launch
./startcf_percy_vertical_xz.sh
```
The source for the Android app is located here: <https://github.com/Lauszus/DroneDraw>.
