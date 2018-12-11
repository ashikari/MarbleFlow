#! /usr/bin/env python

## IMPORTS ##
import rospy
import navigator
import math
import intera_interface 
import threading
from std_msgs.msg import Int32MultiArray
import numpy as np
from sawyer_pykdl import sawyer_kinematics

VERBOSE = 1

class listener():
	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Subscribing topics
		rospy.Subscriber('/ball_talk', Int32MultiArray, self.rx_pos_error)

		# Instance Variables
		self.P_gain = 1e-5;
		self.I_gain = 0;
		self.D_gain = 0;
		self.int_err_x = 0;
		self.int_err_y = 0;
		self.last_err_x = 0;
		self.last_err_y = 0;
		
		# Initialize Sawyer
		rospy.init_node('sawyer_comm', anonymous=True)
		self.limb = intera_interface.Limb('right')
		self.kin = sawyer_kinematics('right')
		navigator.right()

	def rx_pos_error(self, msg):
		# Extract message contents
		if VERBOSE:
			print('I received: ' + str(msg.data[0]) + ',' + str(msg.data[1]))
		err_x = msg.data[0];
		err_y = msg.data[1];

		# Update errors
		self.int_err_x+=err_x;
		self.int_err_y+=err_y;
		der_err_x = self.last_err_x-err_x
		der_err_y = self.last_err_y-err_y
		self.last_err_x = err_x
		self.last_err_y = err_y

		# Calculate endpoint velocity
		ctrl = np.array([self.P_gain*err_x+self.I_gain*self.int_err_x+self.D_gain*der_err_x, self.P_gain*err_y+self.I_gain*self.int_err_y+self.D_gain*der_err_y, 0, 0, 0, 0])

		# Calculate joint velocities from inverse kinematics
		q = np.matmul(self.kin.jacobian_pseudo_inverse(),ctrl)
		if VERBOSE:
			print('\nq = ' + str(q))

		# Send joint velocity command to Sawyer
		# self.limb.set_joint_velocities(q)

if __name__ == '__main__':
	listener()