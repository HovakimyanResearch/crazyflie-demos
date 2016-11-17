#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, TransformStamped
import sys

from crazyflie_driver.srv import UpdateParams

pos_veh0_x = 0
pos_veh0_y = 0
pos_veh0_z = 0

pos_veh1_x = 0
pos_veh1_y = 0
pos_veh1_z = 0

pos_veh2_x = 0
pos_veh2_y = 0
pos_veh2_z = 0


def get_veh0_pos(data):
    global pos_veh0_x, pos_veh0_y, pos_veh0_z
    if pos_veh0_x == 0 and pos_veh0_y == 0 and pos_veh0_z == 0:
        pos_veh0_x = data.transform.translation.x
        pos_veh0_y = data.transform.translation.y
        pos_veh0_z = data.transform.translation.z
        print('Initial position 0:', pos_veh0_x, pos_veh0_y, pos_veh0_z)


def get_veh1_pos(data):
    global pos_veh1_x, pos_veh1_y, pos_veh1_z
    if pos_veh1_x == 0 and pos_veh1_y == 0 and pos_veh1_z == 0:
        pos_veh1_x = data.transform.translation.x
        pos_veh1_y = data.transform.translation.y
        pos_veh1_z = data.transform.translation.z
        print('Initial position 1:', pos_veh1_x, pos_veh1_y, pos_veh1_z)


def get_veh2_pos(data):
    global pos_veh2_x, pos_veh2_y, pos_veh2_z
    if pos_veh2_x == 0 and pos_veh2_y == 0 and pos_veh2_z == 0:
        pos_veh2_x = data.transform.translation.x
        pos_veh2_y = data.transform.translation.y
        pos_veh2_z = data.transform.translation.z
        print('Initial position 2:', pos_veh2_x, pos_veh2_y, pos_veh2_z)


class Trajectory:
    def __init__(self, str_csv_file_name, str_veh_name, veh_number):
        # Open a CSV file and save data in a numpy array
        self.data = np.genfromtxt('agents/' + str_csv_file_name, delimiter=',')

        # Scale the DATA in time and space
        self.max_time_index = self.data[0][-1]
        self.total_path_time = 30.0  #[sec] total path flight time in real time
        self.scale_time = self.total_path_time/self.max_time_index

        #self.max_x = np.max(np.abs(self.data[1]))
        #self.max_y = np.max(np.abs(self.data[2]))
        self.pos_limit = 3 # Limit to +- 1.5 meters
        #self.scale_x = self.pos_limit/self.max_x
        #self.scale_y = self.pos_limit/self.max_y

        #self.scale_pos = min(self.scale_x, self.scale_y) # Find the minimum scaling

        # TODO: Do not hardcode scaling
        #self.scale_time = 1
        self.scale_pos = 0.012178290168060404

        print(str_veh_name, 'Scaling', self.scale_time, self.scale_pos)

        self.waypoint_data = np.array([self.data[0]*self.scale_time, self.data[1]*self.scale_pos - self.pos_limit, self.data[2]*self.scale_pos]) # Scale and add offset
        self.x_init = self.waypoint_data[1][0]
        self.y_init = self.waypoint_data[2][0]
        self.z_init = 1 # Fly at 1 m
        self.x_term = self.waypoint_data[1][-1]
        self.y_term = self.waypoint_data[2][-1]

        # Initialize ROS publisher
        self.str_veh_name = str_veh_name
        self.pos_tgt_msg = Twist()
        self.pub = rospy.Publisher('/' + self.str_veh_name + '/cmd_path', Twist, queue_size=1)
        self.init_ROS_time = rospy.Time.now()
        self.current_state = 0
        self.done = 0
        self.turn_leds_off()
        self.set_pathfollowing_mode(1)

        # Calculate offset compared to the first vehicle
        if veh_number == 2:
            self.x_offset = pos_veh1_x - pos_veh0_x
            self.y_offset = pos_veh1_y - pos_veh0_y
            self.z_offset = pos_veh1_z - pos_veh0_z
        elif veh_number == 3:
            self.x_offset = pos_veh2_x - pos_veh0_x
            self.y_offset = pos_veh2_y - pos_veh0_y
            self.z_offset = pos_veh2_z - pos_veh0_z
        else:
            self.x_offset = 0
            self.y_offset = 0
            self.z_offset = 0
        print(self.str_veh_name, 'Offset', self.x_offset, self.y_offset, self.z_offset)

    def set_parameters(self, cmds, values):
        """See: https://wiki.bitcraze.io/projects:crazyflie2:expansionboards:ledring#parameters"""
        for cmd, values in zip(cmds, values):
            rospy.set_param('/' + self.str_veh_name + '/' + cmd, values)
        rospy.wait_for_service('/' + self.str_veh_name + '/update_params')
        try:
            update_params = rospy.ServiceProxy('/' + self.str_veh_name + '/update_params', UpdateParams)
            update_params(cmds)
        except rospy.ServiceException as e:
            print(self.str_veh_name, 'Service call failed: %s' % e)

    def turn_leds_on(self):
        print(self.str_veh_name, 'Turn LEDs on')
        self.set_parameters(['ring/headlightEnable', 'ring/effect'], [0, 2])

    def turn_leds_off(self):
        print(self.str_veh_name, 'Turn LEDs off')
        self.set_parameters(['ring/headlightEnable', 'ring/effect'], [0, 0])

    def set_pathfollowing_mode(self, val):
        print(self.str_veh_name, 'Set PF mode', val)
        self.set_parameters(['pathfollowing/PF_mode'], [val])

    def publishROS(self):
        if self.done:
            return

        t = rospy.Time.now().to_sec() - self.init_ROS_time.to_sec()

        # Add take off
        if t < 5:
            if self.current_state == 0:
                self.current_state = 1
                print(self.str_veh_name, 'Take off')

            self.pos_tgt_msg.linear.x = 0.0
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = min(max(0.6*t, 0.0), self.z_init)

        # Move to the initial position
        elif 5 <= t < 10:
            if self.current_state == 1:
                self.current_state = 2
                print(self.str_veh_name, 'Move to the initial position')

            self.pos_tgt_msg.linear.x = (t-5)/5 * (self.x_init - self.x_offset)
            self.pos_tgt_msg.linear.y = (t-5)/5 * (self.y_init - self.y_offset)
            self.pos_tgt_msg.linear.z = self.z_init - self.z_offset

        # Position Holding
        elif 10 <= t < 15:
            if self.current_state == 2:
                self.current_state = 3
                print(self.str_veh_name, 'Position hold')

            self.pos_tgt_msg.linear.x = self.x_init - self.x_offset
            self.pos_tgt_msg.linear.y = self.y_init - self.y_offset
            self.pos_tgt_msg.linear.z = self.z_init - self.z_offset

        # Path Flight #
        elif 15 <= t < 15 + self.total_path_time:
            if self.current_state == 3:
                self.current_state = 4
                print(self.str_veh_name, 'Path flight')
                self.turn_leds_on()

            t_path = t - 15
            self.pos_tgt_msg.linear.x = np.interp(t_path, self.waypoint_data[0], self.waypoint_data[1]) - self.x_offset
            self.pos_tgt_msg.linear.y = np.interp(t_path, self.waypoint_data[0], self.waypoint_data[2]) - self.y_offset
            self.pos_tgt_msg.linear.z = self.z_init - self.z_offset

            # print(t_path, self.pos_tgt_msg.linear.x, self.pos_tgt_msg.linear.y)

        # Position Hold before Landing #
        elif 15 + self.total_path_time <= t < 20 + self.total_path_time:
            if self.current_state == 4:
                self.current_state = 5
                print(self.str_veh_name, 'Position hold before landing')
                self.turn_leds_off()

            self.pos_tgt_msg.linear.x = self.x_term - self.x_offset
            self.pos_tgt_msg.linear.y = self.y_term - self.y_offset
            self.pos_tgt_msg.linear.z = self.z_init - self.z_offset

        # Landing
        elif t >= self.total_path_time + 20:
            if self.current_state == 5:
                self.current_state = 6
                print(self.str_veh_name, 'Landing')
                self.land_init_x = self.pos_tgt_msg.linear.x
                self.land_init_y = self.pos_tgt_msg.linear.y
                self.land_init_z = self.pos_tgt_msg.linear.z

            t_landing = t - (self.total_path_time + 20)
            self.pos_tgt_msg.linear.x = max(self.land_init_x - 0.2*t_landing, 0)
            self.pos_tgt_msg.linear.y = max(self.land_init_y - 0.2*t_landing, 0)
            self.pos_tgt_msg.linear.z = max(self.land_init_z - 0.1*t_landing, 0.05)
            if self.pos_tgt_msg.linear.x == 0 and self.pos_tgt_msg.linear.y == 0 and self.pos_tgt_msg.linear.z == 0.05: # Wait until it has landed
                self.set_pathfollowing_mode(0)
                self.done = 1

        self.pub.publish(self.pos_tgt_msg)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Please provide drone names as arguments")
        sys.exit(2)
    elif len(sys.argv) > 4:
        print("Please add another entry for a fourth drone")
        sys.exit(2)

    rospy.init_node('publish_trajectory', anonymous=True)
    rate = rospy.Rate(100)

    rospy.Subscriber("/vicon/" + sys.argv[1] + "/" + sys.argv[1], TransformStamped, get_veh0_pos) # Get position of first vehicle
    while pos_veh0_x == 0 and pos_veh0_y == 0 and pos_veh0_z == 0:
        pass # Wait until the initial position has been set

    if len(sys.argv) > 2:
        rospy.Subscriber("/vicon/" + sys.argv[2] + "/" + sys.argv[2], TransformStamped, get_veh1_pos) # Get position of second vehicle
        while pos_veh1_x == 0 and pos_veh1_y == 0 and pos_veh1_z == 0:
            pass # Wait until the initial position has been set

    if len(sys.argv) > 3:
        rospy.Subscriber("/vicon/" + sys.argv[3] + "/" + sys.argv[3], TransformStamped, get_veh2_pos) # Get position of third vehicle
        while pos_veh2_x == 0 and pos_veh2_y == 0 and pos_veh2_z == 0:
            pass # Wait until the initial position has been set

    veh = list()
    for i in range(1, len(sys.argv)):
        print("Initialize", sys.argv[i])
        veh.append(Trajectory('agent%d.csv' % i, sys.argv[i], i)) # Initialize the class objects

    done = False
    while not rospy.is_shutdown() and not done:
        done = True
        for i in range(len(veh)):
            if not veh[i].done:
                done = False # If all are done exit for-loop
            veh[i].publishROS()
        rate.sleep()
    print('All drones has landed')
