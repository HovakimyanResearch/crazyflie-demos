#!/usr/bin/env python

import rospy
import numpy as np
import os, sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped

from crazyflie_driver.srv import UpdateParams

veh = '/q2'
rospy.set_param(veh+'/pathfollowing/PF_mode', 1)
rospy.wait_for_service(veh+'/update_params')
try:
    update_params = rospy.ServiceProxy(veh+'/update_params', UpdateParams)
    update_params(['pathfollowing/PF_mode'])
except rospy.ServiceException as e:
    print('Service Call Failed: %s' % e)

class CraneMode:
    def deadzone(self, x):
        if np.abs(x) <= 0.16:
            return 0
        elif x>0:
            return x - 0.16
        else:
            return x + 0.16

    def getQ2Pos(self, data):
        self.pos_q2_x = data.transform.translation.x
        self.pos_q2_y = data.transform.translation.y
        self.pos_q2_z = data.transform.translation.z

    def getCraneCmd(self, data):
        self.Vel_x_cmd = self.deadzone(data.axes[3])
        self.Vel_y_cmd = self.deadzone(data.axes[4])
        self.Vel_z_cmd = self.deadzone(data.axes[1])
        if self.land == 0:
            self.land = data.buttons[0]
        if self.pickup == 0:
            self.pickup = data.buttons[1]
        if data.buttons[2] == 1:
            self.pickup = 0

    def getPillBoxPos(self,data):
        self.pos_pillbox_x = data.transform.translation.x
        self.pos_pillbox_y = data.transform.translation.y
        self.pos_pillbox_z = data.transform.translation.z

    def publishROS(self):
        t = rospy.Time.now().to_sec() - self.init_ROS_time.to_sec()

        if t < 4 and not self.land and not self.pickup:
            if self.current_state == 0:
                self.current_state = 1
                print('Take off')
            self.pos_tgt_msg.linear.x = 0.0
            self.pos_tgt_msg.linear.y = 0.0
            if t >= 2:
                self.pos_tgt_msg.linear.z = min(max(0.35*(t-2), 0.0), 1.0)
            else:
                self.pos_tgt_msg.linear.z = 0.0
                self.cnt += 1
                if (self.cnt < 10):
                    self.pickup_x = self.pos_pillbox_x - self.pos_q2_x
                    self.pickup_y = self.pos_pillbox_y - self.pos_q2_y
                    self.pickup_z = self.pos_pillbox_z - self.pos_q2_z
                    self.original_z = self.pos_q2_z
                    if self.cnt == 9:
                        print (self.pickup_x, self.pos_q2_y, self.pos_q2_z)

        elif t >= 4 and not self.land and not self.pickup:
            if self.current_state == 1:
                self.current_state = 2
                print('Crane mode')

            self.pos_tgt_msg.linear.x = self.pos_tgt_msg.linear.x - 0.01*self.Vel_x_cmd
            self.pos_tgt_msg.linear.y = self.pos_tgt_msg.linear.y + 0.01*self.Vel_y_cmd
            self.pos_tgt_msg.linear.z = max(self.pos_tgt_msg.linear.z + 0.008*self.Vel_z_cmd, 0.05)

            if self.pos_tgt_msg.linear.x > 3:
                self.pos_tgt_msg.linear.x = 3
            elif self.pos_tgt_msg.linear.x < -3:
                self.pos_tgt_msg.linear.x = -3

            if self.pos_tgt_msg.linear.y > 3:
                self.pos_tgt_msg.linear.y = 2
            elif self.pos_tgt_msg.linear.y < -2:
                self.pos_tgt_msg.linear.y = -2

            if self.pos_tgt_msg.linear.z > 2.5:
                self.pos_tgt_msg.linear.z = 2.5

            #  if np.linalg.norm((self.pickup_x + self.delta_x_pick) - self.pos_tgt_msg.linear.x) < 0.15:
            self.dir_pickup_x = ((self.pickup_x + self.delta_x_pick) - self.pos_tgt_msg.linear.x)/np.linalg.norm((self.pickup_x + self.delta_x_pick) - self.pos_tgt_msg.linear.x)
            #  if np.linalg.norm((self.pickup_y + self.delta_y_pick) - self.pos_tgt_msg.linear.y) < 0.15:
            self.dir_pickup_y = ((self.pickup_y + self.delta_y_pick) - self.pos_tgt_msg.linear.y)/np.linalg.norm((self.pickup_y + self.delta_y_pick) - self.pos_tgt_msg.linear.y)
            #  if np.linalg.norm((self.pickup_z + self.delta_z_pick) - self.pos_tgt_msg.linear.z) < 0.15:
            self.dir_pickup_z = ((self.pickup_z + self.delta_z_pick) - self.pos_tgt_msg.linear.z)/np.linalg.norm((self.pickup_z + self.delta_z_pick) - self.pos_tgt_msg.linear.z)
            self.t_pick_up = t

        elif t>=4 and not self.land and self.pickup:
            if np.abs(self.pos_tgt_msg.linear.x - (self.pickup_x + self.delta_x_pick)) < 0.1:
                self.pos_tgt_msg.linear.x = self.pickup_x + self.delta_x_pick
            else:
                self.pos_tgt_msg.linear.x += 0.0002*(t-self.t_pick_up)*(self.dir_pickup_x)

            if np.abs(self.pos_tgt_msg.linear.y - (self.pickup_y + self.delta_y_pick)) < 0.1:
                self.pos_tgt_msg.linear.y = self.pickup_y + self.delta_y_pick
            else:
                self.pos_tgt_msg.linear.y += 0.0002*(t-self.t_pick_up)*(self.dir_pickup_y)

            if np.abs(self.pos_tgt_msg.linear.z - (self.pickup_z + self.delta_z_pick)) < 0.1:
                self.pos_tgt_msg.linear.z = self.pickup_z + self.delta_z_pick
            else:
                self.pos_tgt_msg.linear.z += 0.00005*(t-self.t_pick_up)*(self.dir_pickup_z)

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
            self.pos_tgt_msg.linear.z = max(self.land_init_z - 0.08*t_landing, 0.1)

            if self.pos_tgt_msg.linear.x == 0 and self.pos_tgt_msg.linear.y == 0 and self.pos_tgt_msg.linear.z <= 0.1: # Wait until it has landed
                    self.done = 1



            #print(self.Vel_x_cmd, self.Vel_y_cmd, self.pos_tgt_msg.linear.x, self.pos_tgt_msg.linear.y)
        print ("Target:",self.pos_tgt_msg.linear.z + self.original_z," True:",self.pos_q2_z)

        self.pub.publish(self.pos_tgt_msg)

    def __init__(self, str_veh_name):
        ### Initialize ROS publisher ###
        self.pos_tgt_msg = Twist()
        self.pub = rospy.Publisher('/' + str_veh_name + '/cmd_path', Twist, queue_size=1)
        self.init_ROS_time = rospy.Time.now()
        #  self.sub = rospy.Subscriber("/q2/joy", Joy, self.getCraneCmd)
        self.sub = rospy.Subscriber(veh+"/joy", Joy, self.getCraneCmd)
        self.obj_sub = rospy.Subscriber("/vicon/table/table",TransformStamped,self.getPillBoxPos)
        #  self.obj_sub = rospy.Subscriber("/vicon/q2/q2",TransformStamped,self.getQ2Pos)
        self.obj_sub = rospy.Subscriber("/vicon"+veh+veh,TransformStamped,self.getQ2Pos)
        self.Vel_x_cmd = 0
        self.Vel_y_cmd = 0
        self.Vel_z_cmd = 0
        #states
        self.current_state = 0
        self.land = 0
        self.done = 0
        self.pickup = 0
        self.pos_pillbox_x = 0.
        self.pos_pillbox_y = 0.
        self.pos_pillbox_z = 0.
        self.delta_x_pick = 0.0
        self.delta_y_pick = 0.0
        self.delta_z_pick = 0.31
        self.pos_q2_x = 0.
        self.pos_q2_y = 0.
        self.pos_q2_z = 0.
        self.pickup_x = 0.
        self.pickup_y = 0.
        self.pickup_z = 0.
        self.dir_pickup_x = 0.1
        self.dir_pickup_y = 0.1
        self.dir_pickup_z = 0.1
        self.t_pick_up = 0.
        self.cnt = 0

if __name__ == '__main__':
    rospy.init_node('publish_trajectory', anonymous=True)
    rate = rospy.Rate(100)
    #  veh1 = CraneMode('q2')    ### Initialize the class object
    veh1 = CraneMode(veh[1:])
    while not rospy.is_shutdown() and not veh1.done:
        veh1.publishROS()
        rate.sleep()


