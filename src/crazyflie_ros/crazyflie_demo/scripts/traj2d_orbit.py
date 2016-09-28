#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as la
import h5py
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class Trajectory:
    def __init__(self, str_mat_file_name, str_veh_name):
        ### Open a mat file and save data in a numpy array ###  
        self.file = h5py.File('/home/stargazer/catkin_ws/src/crazyflie_ros/crazyflie_demo/'+str_mat_file_name, 'r')
        self.data = np.array(self.file.get('ans')).T*0.5    # data[0] = TIMEs, data[1] = Xs, data[2] = Ys, data[3] = Zs
        self.init_xyz = np.array([self.data[1][0], self.data[2][0], self.data[3][0]])
        self.final_xyz = np.array([self.data[1][-1], self.data[2][-1], self.data[3][-1]])
         
        ### Initialize ROS publisher ###
        self.pos_tgt_msg = Twist()
        self.pub = rospy.Publisher(  '/'+ str_veh_name + '/cmd_path', Twist, queue_size=1)
        self.init_ROS_time =  rospy.Time.now()
        self.now_ROS_time = rospy.Time.now()
        self.time_now_in_sec = 0.0
        self.xyz_previcous = np.zeros(3)
        self.xyz_now = np.zeros(3)
        

    def publishROS(self):
        self.now_ROS_time = rospy.Time.now()
        self.time_now_in_sec = self.now_ROS_time.to_sec() - self.init_ROS_time.to_sec()
        t = self.time_now_in_sec
        if t < 77:
            self.pos_tgt_msg.linear.x = np.interp(t/2, self.data[0], self.data[1]) - self.init_xyz[0]
            self.pos_tgt_msg.linear.y = np.interp(t/2, self.data[0], self.data[2]) - self.init_xyz[1]
            self.pos_tgt_msg.linear.z = np.interp(t/2, self.data[0], self.data[3])

        else:
            t_landing = t - (77)
            self.pos_tgt_msg.linear.z = min(max(self.final_xyz[2] - 0.1*t_landing, 0.0),2.0) 

        self.pub.publish(self.pos_tgt_msg)
        

if __name__ == '__main__':
    rospy.init_node('publish_trajectory', anonymous=True)
    rate = rospy.Rate(15)   
    veh1 = Trajectory('Traj_2D_UAV1.mat','danny')    ### Initialize the class object
    veh2 = Trajectory('Traj_2D_UAV2.mat','percy')    ### Initialize the class object

    while not rospy.is_shutdown():
        veh1.publishROS()
        veh2.publishROS()
        rate.sleep()


