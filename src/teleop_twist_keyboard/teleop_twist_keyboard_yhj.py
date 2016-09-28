#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty, math

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
                     I
                  J  K  L                  
 A
 Z

CTRL-C to quit
"""

tgtBindings = {
		'i':(1,0,0),
		'k':(-1,0,0),
		'l':(0,1,0),
		'j':(0,-1,0),
		'a':(0,0,1),
		'z':(0,0,-1),
	       }

serviceBindings = {      # To be added
		'1':(1),
		'2':(2),
		'3':(3),
	       }


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def deadzone(tgt_Value, deadzone):
	deadzone = math.fabs(deadzone)
	if (tgt_Value < deadzone)&(tgt_Value > - deadzone):
		tgt_Value = 0
	else:
		tgt_Value = tgt_Value

	return tgt_Value 

tgtX = 0
tgtY = 0
tgtZ = 0

status = 0


def tgt(tgtX,tgtY,tgtZ):
	return "currently:\ttgtX %s\ttgtY %s \ttgtZ %s" % (tgtX,tgtY,tgtZ)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/percy/cmd_path', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	

	try:
		print msg
		print tgt(tgtX,tgtY,tgtZ)
		while(1):
			key = getKey()
			if key in tgtBindings.keys():
				tgtX = deadzone(tgtX + tgtBindings[key][0]*0.1, 0.05)
				tgtY = deadzone(tgtY + tgtBindings[key][1]*0.1, 0.05)
				tgtZ = deadzone(tgtZ + tgtBindings[key][2]*0.1, 0.05)
				y = tgtBindings[key][1]
				z = tgtBindings[key][2]

				tgtX = deadzone(tgtX, 0.1)

				print tgt(tgtX,tgtY,tgtZ)

				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:

				if (key == '\x03'):
					break
			rate = rospy.Rate(10) # 10hz		
			twist = Twist()
			twist.linear.x = tgtX; twist.linear.y = tgtY; twist.linear.z = tgtZ;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
			pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
