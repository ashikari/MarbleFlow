#! /usr/bin/env python

## IMPORTS ##
import rospy
import navigator
import math
import intera_interface 
import threading
from std_msgs.msg import (
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16
)

class listener():
	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Subscribing topics
		rospy.Subscriber('/chatter', String, self.rx_pos_error)
		
		# Initialize Sawyer
		rospy.init_node('Sawyer_Sparrow_comm_node', anonymous=True)
		self.limb = intera_interface.Limb('right')
		navigator.right()

	# Publishes message to ROSTopic to be receieved by JavaScript
	def pub_cmd(self, cmd):
		msg = Int32()
		msg.data = cmd
		self.cmd2browser.publish(msg)
		if verbose: rospy.loginfo("I sent: " + str(cmd))

	## LISTENER EVENTS ##
	# Runs whenever message is received on main ROSTopic from JavaScript
	def rx_pos_error(self, msg):
		print(msg.data)

if __name__ == '__main__':
	listener()