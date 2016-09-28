#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as la
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def const_speed_linear_move(t, np_array_initial_pos, np_array_final_pos, time_length):
    x_tgt = np_array_initial_pos + (np_array_final_pos - np_array_initial_pos)*t/time_length
    return x_tgt

class path:
    def __init__(self, np_array_x_init, np_array_x_final, start_time, end_time):
        self.init_pos = np_array_x_init
        self.final_pos = np_array_x_final
        self.start_time =start_time
        self.end_time =end_time

class PathCmd:
    def __init__(self):    
        self.pos_meas = np.zeros(3)
        self.pos_tgt = np.zeros(3)
        self.pos_tgt_msg = Twist()
        self.pub1 = rospy.Publisher('/percy/cmd_path', Twist, queue_size=1)
        self.pub2 = rospy.Publisher('/danny/cmd_path', Twist, queue_size=1)
        self.init_ROS_time =  rospy.Time.now()
        self.now_ROS_time = rospy.Time.now()
        self.time_now_in_sec = 0.0        

        ## each path is consist of initial position, final position, path_time ##
        self.path1 = path(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 2.0]), 0.0,     5.0)
        ##### hovering 5 sec #####
        self.path2 = path(np.array([0.0, 0.0, 2.0]), np.array([0.5, 0.0, 2.0]), 10.0,  15.0)
        ##### hovering 5 sec #####
        self.path3 = path(np.array([0.5, 0.0, 2.0]), np.array([0.5, 0.5, 2.0]), 20.0,  25.0)
        ##### hovering 5 sec #####
        self.path4 = path(np.array([0.5, 0.5, 2.0]), np.array([0.0, 0.5, 2.0]), 30.0,  35.0)
        ##### hovering 5 sec #####
        self.path5 = path(np.array([0.0, 0.5, 2.0]), np.array([0.0, 0.0, 2.0]), 40.0,  45.0)
        ##### hovering 5 sec #####
        self.path6 = path(np.array([0.0, 0.0, 2.0]), np.array([0.0, 0.0, 0.0]), 50.0,  55.0)
        


    def getPos(self, msg_data):    
        self.pos_meas = np.array([msg_data.linear.x, msg_data.linear.y, msg_data.linear.z])


    def publishROS(self):
        self.now_ROS_time = rospy.Time.now()
        self.time_now_in_sec = self.now_ROS_time.to_sec() - self.init_ROS_time.to_sec()
        t = self.time_now_in_sec
        
        if (t <= 5.0):
            self.pos_tgt = const_speed_linear_move(t - self.path1.start_time, self.path1.init_pos, self.path1.final_pos, 5)
        elif ( t >5.0 and t <= 10.0):
            self.pos_tgt = self.path1.final_pos
        elif ( t >10.0 and t <=15.0 ):
            self.pos_tgt = const_speed_linear_move(t - self.path2.start_time, self.path2.init_pos, self.path2.final_pos, 5)
        elif ( t >15.0 and t <=20.0 ):
            self.pos_tgt = self.path2.final_pos
        elif ( t >20.0 and t <=25.0 ):
            self.pos_tgt = const_speed_linear_move(t - self.path3.start_time, self.path3.init_pos, self.path3.final_pos, 5)
        elif ( t >25.0 and t <=30.0 ):
            self.pos_tgt = self.path3.final_pos
        elif ( t >30.0 and t <=35.0 ):
            self.pos_tgt = const_speed_linear_move(t - self.path4.start_time, self.path4.init_pos, self.path4.final_pos, 5)
        elif ( t >35.0 and t <=40.0 ):
            self.pos_tgt = self.path4.final_pos
        elif ( t >40.0 and t <=45.0 ):
            self.pos_tgt = const_speed_linear_move(t - self.path5.start_time, self.path5.init_pos, self.path5.final_pos, 5)
        elif ( t >45.0 and t <=50.0 ):
            self.pos_tgt = self.path5.final_pos
        elif ( t >50.0 and t <=55.0 ):
            self.pos_tgt = const_speed_linear_move(t - self.path6.start_time, self.path6.init_pos, self.path6.final_pos, 5)
        elif ( t >55.0 and t <=60.0 ):
            self.pos_tgt = self.path6.final_pos
        

        self.pos_tgt_msg.linear.x = self.pos_tgt[0]
        self.pos_tgt_msg.linear.y = self.pos_tgt[1]
        self.pos_tgt_msg.linear.z = self.pos_tgt[2]
        self.pub1.publish(self.pos_tgt_msg)
        self.pub2.publish(self.pos_tgt_msg)
        

if __name__ == '__main__':
    rospy.init_node('publish_trajectory', anonymous=True)
    rate = rospy.Rate(50)
   
    path = PathCmd()    ### Initialize the class object

    while not rospy.is_shutdown():
        path.publishROS()
        rate.sleep()


