#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class CraneMode:

    def getCraneCmd(self, data):
        self.Vel_x_cmd = data.axes[3]
        self.Vel_y_cmd = data.axes[4]
	if self.land == 0:
            self.land = data.buttons[0]
 
    def publishROS(self):
        t = rospy.Time.now().to_sec() - self.init_ROS_time.to_sec()
        
        if t < 5 and not self.land:
	    if self.current_state == 0:
		self.current_state = 1
		print('Take off')
            self.pos_tgt_msg.linear.x = 0.0
            self.pos_tgt_msg.linear.y = 0.0
            self.pos_tgt_msg.linear.z = min(max(0.6*t, 0.0), 1)

        elif t >= 5 and not self.land:
            if self.current_state == 1:
		self.current_state = 2
		print('Crane mode')
            self.pos_tgt_msg.linear.x = self.pos_tgt_msg.linear.x - 0.01*self.Vel_x_cmd
            self.pos_tgt_msg.linear.y = self.pos_tgt_msg.linear.y + 0.01*self.Vel_y_cmd 
            self.pos_tgt_msg.linear.z = 1

	    if self.pos_tgt_msg.linear.x > 2:
		self.pos_tgt_msg.linear.x = 2
            elif self.pos_tgt_msg.linear.x < -2:
		self.pos_tgt_msg.linear.x = -2

	    if self.pos_tgt_msg.linear.y > 2:
		self.pos_tgt_msg.linear.y = 2
            elif self.pos_tgt_msg.linear.y < -2:
		self.pos_tgt_msg.linear.y = -2

	else:
	    if self.current_state == 2:
                self.current_state = 3
                print('Landing')
		self.land_init_time = t
                self.land_init_x = self.pos_tgt_msg.linear.x
         	self.land_init_y = self.pos_tgt_msg.linear.y
	        self.land_init_z = self.pos_tgt_msg.linear.z

            t_landing = t - self.land_init_time
            self.pos_tgt_msg.linear.x = max(self.land_init_x - 0.2*t_landing, 0)
            self.pos_tgt_msg.linear.y = max(self.land_init_y - 0.2*t_landing, 0)
            self.pos_tgt_msg.linear.z = max(self.land_init_z - 0.1*t_landing, 0.05)

            if self.pos_tgt_msg.linear.x == 0 and self.pos_tgt_msg.linear.y == 0 and self.pos_tgt_msg.linear.z == 0.05: # Wait until it has landed
                    self.done = 1

	    #print(self.Vel_x_cmd, self.Vel_y_cmd, self.pos_tgt_msg.linear.x, self.pos_tgt_msg.linear.y)

        self.pub.publish(self.pos_tgt_msg)

    def __init__(self, str_veh_name):
        ### Initialize ROS publisher ###
        self.pos_tgt_msg = Twist()
        self.pub = rospy.Publisher('/' + str_veh_name + '/cmd_path', Twist, queue_size=1)
        self.init_ROS_time = rospy.Time.now()
        self.sub = rospy.Subscriber("/joy", Joy, self.getCraneCmd)
	self.Vel_x_cmd = 0
	self.Vel_y_cmd = 0
	self.current_state = 0
	self.land = 0
	self.done = 0
        

if __name__ == '__main__':
    rospy.init_node('publish_trajectory', anonymous=True)
    rate = rospy.Rate(100)   
    veh1 = CraneMode('percy')    ### Initialize the class object

    while not rospy.is_shutdown() and not veh1.done:
        veh1.publishROS()
        rate.sleep()


