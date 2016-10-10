# Code for the Pygmalion Festival

First download and install ROS Indigo: <http://wiki.ros.org/indigo/Installation/Ubuntu>.

See the following repository for instructions on how to compile ROS for Mac OS X: <https://github.com/mikepurvis/ros-install-osx>.

In order for the Vicon bridge to work you need to place the dylibs in /usr/lib and mark them as executable:
```
sudo cp libViconDataStreamSDK_CPP.dylib /usr/libsudo cp libDebugServices.dylib /usr/libsudo chmod 755 /usr/lib/libViconDataStreamSDK_CPP.dylib sudo chmod 755 /usr/lib/ libDebugServices.dylib
```

Then build this ROS packages:
```
catkin_make
```

### DroneDraw

Run the following commands in two separate terminals in order run the drone draw demo:
```
source devel/setup.bash && roslaunch crazyflie_driver crazyflie_percy.launch vicon_ip:=192.168.1.110:801
./startcf_drone_draw.sh percy
```
Where ```vicon_ip:=192.168.1.110:801``` is IP adress of the computer running Vicon.

The source for the Android app is located here: <https://github.com/Lauszus/DroneDraw>. The Android app will upload the path to your Dropbox in the ```~/Dropbox/Apps/DroneDraw``` directory.

To generate a new trajectory based on the path generated from the Android app:
```
./GeneratePath.py -i ~/Dropbox/Apps/DroneDraw/path.csv -o ~/Dropbox/Apps/DroneDraw/IRL_path.csv
```
This will generate a trajectory using Bezier curves and generate timestamps based on constant velocity. Currently the demo expect the generated path to be located in that particular folder.

Add the argument ```-p``` in order to plot the generated path. The argument ```-w``` makes the script wait until the path has changed. This is useful when using it in a live demo, as it will automatically detect when a new path is uploaded to Dropbox and then calculate a trajectory automatically.

### Crane demo
Start crazyflie in center of the room.

1. Setup
  * Connect to Cisco11477 wifi or Ethernet
  * Plug in xbox controller. Press center button of controller to turn on
  * Plug in Crazyradio PA dongle
  * Point front (red/green lights) of vehicle towards door
  * Power vehicle

2. Edit Config Files
  * Drone Name
  [repo_location]/src/crazyflie_ros/crazyflie_demo/scripts/Crane.py
  In the main function you must choose which version of veh1. So far we have either percy or q2. One option will be enabled and the other will be commented.

  * Set Vicon Computer IP (if not already correct)
     ```
     [repo_location]/src/vicon_bridge/launch/vicon.launch
     ```

3. Start ROS
 * Open terminal and run:
   ```
   roscore
   ```

4. Enable Joystick
  * Open terminal and run:
  ```
  rosrun joy joy_node
  ```

5. Source bash script for and run roslaunch
  * Open terminal and run:
  ```
  cd [repo_location]
  source devel/setup.bash
  roslaunch crazyflie_driver crazyflie_DRONENAME.launch
  ```
  vicon data should be streaming

6. Launch Crazyflie
 * Open Terminal
  ```
  cd [repo_location]
  ./start[cf|q2]_crane.sh
  ```
7. Ctrl + C to stop
