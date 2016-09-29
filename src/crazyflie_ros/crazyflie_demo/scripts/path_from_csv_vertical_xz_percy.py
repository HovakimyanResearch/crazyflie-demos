#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import os


class Trajectory:
    def __init__(self, str_csv_file_name, str_veh_name):
        ### Open a CSV file and save data in a numpy array ###
        self.df = np.genfromtxt(os.path.expanduser('~/Dropbox/Data/' + str_csv_file_name), delimiter=',')
        self.data = self.df.T # data[0] = TIMEs, data[1] = Xs, data[2] = Ys

        ### Scale the DATA in time and space
        #self.max_time_index = self.data[0][-1]
        #self.total_path_time = 60.0  #[sec] total path flight time in real time
        #self.scale_time = self.total_path_time/self.max_time_index

        self.total_path_time = self.data[0][-1]
        self.scale_time = 1.0

        self.scale_x = 1.0 # [m/pixl]
        self.scale_y = 1.0 # [m/pixl]
        self.waypoint_data = np.array([self.data[0]*self.scale_time, self.data[1]*self.scale_x, self.data[2]*self.scale_y])
        self.x_init = self.waypoint_data[1][0]
        self.z_init = self.waypoint_data[2][0]
        self.x_term = self.waypoint_data[1][-1]
        self.z_term = self.waypoint_data[2][-1]

        ### Initialize ROS publisher ###
        self.pos_tgt_msg = Twist()
        self.pub = rospy.Publisher('/' + str_veh_name + '/cmd_path', Twist, queue_size=1)
        self.init_ROS_time = rospy.Time.now()
        self.current_state = 0
        self.done = 0

    def publishROS(self):
        self.now_ROS_time = rospy.Time.now()
        t = self.now_ROS_time.to_sec() - self.init_ROS_time.to_sec()

        # Add take off #
        if t < 5:
            if self.current_state == 0:
                self.current_state = 1
                print('Take off')
            self.pos_tgt_msg.linear.x = 0.0
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = min(max(0.6*t, 0.0), 0.5)

        # Move to the initial position #
        elif (t>=5) and (t < 10):
            if self.current_state == 1:
                self.current_state = 2
                print('Move to the initial position')
            self.pos_tgt_msg.linear.x = (t-5)/5 * self.x_init
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = (t-5)/5 * self.z_init + 0.5

        # Position Holding #
        elif (t>=10) and (t < 15):
            if self.current_state == 2:
                self.current_state = 3
                print('Position Holding')
            self.pos_tgt_msg.linear.x = self.x_init
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = self.z_init +0.5

        # Path Flight #
        elif (t>=15) and (t < 15 + self.total_path_time):
            if self.current_state == 3:
                self.current_state = 4
                print('Path Flight')
            t_path = t - 15
            self.pos_tgt_msg.linear.x = np.interp(t_path, self.waypoint_data[0], self.waypoint_data[1])
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = np.interp(t_path, self.waypoint_data[0], self.waypoint_data[2])+0.5

            # print(t_path, self.pos_tgt_msg.linear.x, self.pos_tgt_msg.linear.y)

        # Position Hold before Landing #
        elif (t>=15 + self.total_path_time) and (t < 20 + self.total_path_time):
            if self.current_state == 4:
                self.current_state = 5
                print('Position Hold before Landing')
            self.pos_tgt_msg.linear.x = self.x_term
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = self.z_term+0.5

        # Landing
        elif (t >= self.total_path_time+20):
            if self.current_state == 5:
                self.current_state = 6
                print('Landing')
            t_landing = t - (self.total_path_time + 20)
            self.pos_tgt_msg.linear.x = max(self.x_term - 0.1*t_landing, 0)
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = max(self.z_term + 0.5 - 0.1*t_landing, 0.05)
            if self.pos_tgt_msg.linear.x == 0 and self.pos_tgt_msg.linear.z == 0.05: # Wait until it has landed
                    self.done = 1

        self.pub.publish(self.pos_tgt_msg)


if __name__ == '__main__':
    rospy.init_node('publish_trajectory', anonymous=True)
    rate = rospy.Rate(100)
    veh1 = Trajectory('IRL_path.csv','percy')    ### Initialize the class object

    while not rospy.is_shutdown() and not veh1.done:
        veh1.publishROS()
        rate.sleep()
