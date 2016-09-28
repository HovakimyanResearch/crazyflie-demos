#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as la
import h5py
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

tgt_altitude = 0.6
time_scale = 2.0   # 2.0 times slower

class Trajectory:
    def __init__(self, str_mat_file_name, str_veh_name, z_offset):
        ### Open a mat file and save data in a numpy array ###  
        self.file = h5py.File('/home/stargazer/catkin_ws/src/crazyflie_ros/crazyflie_demo/'+str_mat_file_name, 'r')
        self.data = np.array(self.file.get('ans')).T *0.5   # data[0] = TIMEs, data[1] = Xs, data[2] = Ys, data[3] = Zs
        self.total_path_time = self.data[0][-1]
        self.x_term = self.data[1][-1]
        self.y_term = self.data[2][-1]
        self.z_offset = z_offset
         
        ### Initialize ROS publisher ###
        self.pos_tgt_msg = Twist()
        self.pub = rospy.Publisher(  '/'+ str_veh_name + '/cmd_path', Twist, queue_size=1)
        self.init_ROS_time =  rospy.Time.now()
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
            self.pos_tgt_msg.linear.z = min(max(0.3*t, 0.0),tgt_altitude) + self.z_offset

        # Path Flight # 
        elif (t>=5) and (t < 5 + self.total_path_time*time_scale):
            t_path = t - 5
            self.pos_tgt_msg.linear.x = np.interp(t_path/time_scale, self.data[0], self.data[1])
            self.pos_tgt_msg.linear.y = np.interp(t_path/time_scale, self.data[0], self.data[2])
            self.pos_tgt_msg.linear.z = tgt_altitude + self.z_offset

        # Landing
        elif (t >= self.total_path_time*time_scale+5):
            t_landing = t - (self.total_path_time*time_scale + 5)
            self.pos_tgt_msg.linear.x = self.x_term
            self.pos_tgt_msg.linear.y = self.y_term
            self.pos_tgt_msg.linear.z = min(max(tgt_altitude - 0.2*t_landing, 0.0),2.0) + self.z_offset
            
        else: 
            self.pos_tgt_msg.linear.x = 0.0
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = 0.0

        self.pub.publish(self.pos_tgt_msg)
        

if __name__ == '__main__':
    rospy.init_node('publish_trajectory', anonymous=True)
    rate = rospy.Rate(100)   
    veh1 = Trajectory('q1_trj_vel_add.mat','danny', 0.15)    ### Initialize the class object
    veh2 = Trajectory('q2_trj_vel_add.mat','percy', -0.15)    ### Initialize the class object

    while not rospy.is_shutdown():
        veh1.publishROS()
        veh2.publishROS()
        rate.sleep()


