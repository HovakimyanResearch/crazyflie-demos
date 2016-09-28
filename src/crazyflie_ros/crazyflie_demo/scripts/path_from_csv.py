#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as la

import pandas as pd

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class Trajectory:
    def __init__(self, str_csv_file_name, str_veh_name):
        ### Open a CSV file and save data in a numpy array ###  
        self.df = pd.read_csv('/home/stargazer/Dropbox/Apps/DroneDraw/'+str_csv_file_name , sep=',', header=None)
        self.data = np.array(self.df).T    # data[0] = TIMEs, data[1] = Xs, data[2] = Ys, data[3] = Zs
        self.max_time_index = self.data[0][-1]
        
        ### Scale the DATA in time and space
        self.total_path_time = 60.0  #[sec] total path flight time in real time
        self.scale_time = self.total_path_time/self.max_time_index
        
        self.scale_x = 1.0      #[m/pixl]
        self.scale_y = 1.0      #[m/pixl]
        self.waypoint_data = np.array([self.data[0]*self.scale_time, self.data[1]*self.scale_x, self.data[2]*self.scale_y])
        self.x_init = self.waypoint_data[1][0]
        self.y_init = self.waypoint_data[2][0]
        self.x_term = self.waypoint_data[1][-1]
        self.y_term = self.waypoint_data[2][-1]
              
        ### Initialize ROS publisher ###
        self.pos_tgt_msg = Twist()
        self.pub = rospy.Publisher(  '/'+ str_veh_name + '/cmd_path', Twist, queue_size=1)
        self.init_ROS_time = rospy.Time.now()
        self.now_ROS_time = rospy.Time.now()
        self.time_now_in_sec = 0.0  

    def publishROS(self):
        self.now_ROS_time = rospy.Time.now()
        self.time_now_in_sec = self.now_ROS_time.to_sec() - self.init_ROS_time.to_sec()
        t = self.time_now_in_sec
        
        # Add take off #
        if t < 5:
            self.pos_tgt_msg.linear.x = 0.0
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = min(max(0.6*t, 0.0),2.0)
            
        # Move to the initial position #        
        elif (t>=5) and (t < 10):
            self.pos_tgt_msg.linear.x = (t-5)/5 * self.x_init
            self.pos_tgt_msg.linear.y = (t-5)/5 * self.y_init
            self.pos_tgt_msg.linear.z = 2.0
            
        # Position Holding #        
        elif (t>=10) and (t < 15):
            self.pos_tgt_msg.linear.x = self.x_init
            self.pos_tgt_msg.linear.y = self.y_init
            self.pos_tgt_msg.linear.z = 2.0
            
        # Path Flight # 
        elif (t>=15) and (t < 15 + self.total_path_time):
            t_path = t - 15
            self.pos_tgt_msg.linear.x = np.interp(t_path, self.waypoint_data[0], self.waypoint_data[1])
            self.pos_tgt_msg.linear.y = np.interp(t_path, self.waypoint_data[0], self.waypoint_data[2])
            self.pos_tgt_msg.linear.z = 2.0
            
            # print(t_path, self.pos_tgt_msg.linear.x, self.pos_tgt_msg.linear.y)
            
        # Position Hold before Landing # 
        elif (t>=15 + self.total_path_time) and (t < 20 + self.total_path_time):
            self.pos_tgt_msg.linear.x = self.x_term
            self.pos_tgt_msg.linear.y = self.y_term
            self.pos_tgt_msg.linear.z = 2.0
            
        # Landing
        elif (t >= self.total_path_time+20):
            t_landing = t - (self.total_path_time + 20)
            self.pos_tgt_msg.linear.x = self.x_term
            self.pos_tgt_msg.linear.y = self.y_term
            self.pos_tgt_msg.linear.z = min(max(2.0 - 0.6*t_landing, 0.0),2.0)
            
        else: 
            self.pos_tgt_msg.linear.x = 0.0
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = 0.0

        self.pub.publish(self.pos_tgt_msg)
        

if __name__ == '__main__':
    rospy.init_node('publish_trajectory', anonymous=True)
    rate = rospy.Rate(100)   
    veh1 = Trajectory('IRL.csv','danny')    ### Initialize the class object

    while not rospy.is_shutdown():
        veh1.publishROS()
        rate.sleep()


