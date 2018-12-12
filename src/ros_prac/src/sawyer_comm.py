#! /usr/bin/env python

## IMPORTS ##
import rospy
import navigator
import math
import intera_interface 
import go_to
import threading
from std_msgs.msg import Int32MultiArray
import numpy as np
from numpy import linalg as la
from sawyer_pykdl import sawyer_kinematics

VERBOSE = 0

class listener():
	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Instance Variables
		self.P_gain = 1.25e-3;
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
		joint_positions = go_to.joint_angle_arg(joint_angles = [math.pi/2, -math.pi/4, 0, math.pi/2-0.3, 0, math.pi/4+0.3, 2.46 ], speed_ratio = .225, accel_ratio = 0.1)
		go_to.joint_angles(joint_positions)
		joint_angles = self.limb.joint_angles()
		q = np.zeros(7)
		for j in range(0,7):
			q[j] = joint_angles['right_j' + str(j)]
		print(np.matmul(self.kin.jacobian(),q))
		self.lastTime = rospy.get_time()

		# Subscribing topics
		rospy.Subscriber('/ball_error', Int32MultiArray, callback=self.rx_pos_error, queue_size=1)

		navigator.right()

	def rx_pos_error(self, msg):
		self.limb.set_command_timeout(1)
		# Extract message contents
		if VERBOSE:
			print('\nI received: ' + str(msg.data[0]) + ',' + str(msg.data[1]))
		err_x = -msg.data[1];
		err_y = -msg.data[0];

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
		inv_J = self.kin.jacobian_pseudo_inverse()
		if (la.norm(inv_J, 'fro')>90):
			print('NEAR SINGULAR')
			rospy.signal_shutdown('Quit')
		endpoint_pos = self.limb.endpoint_pose()['position'];
		if (endpoint_pos[0]>0.69 or endpoint_pos[0]<-0.27 or endpoint_pos[1]>0.85 or endpoint_pos[1]<0.31):
			print('AT BOUNDARY')
			rospy.signal_shutdown('Quit')
		q = np.matmul(inv_J,ctrl)
		if VERBOSE:
			print('m = ' + str(msg.data))
			print('e = (' + str(err_x) + ", " + str(err_y) + ")")
			print('x = ' + str(ctrl))
			print('q = ' + str(q))

		# Send joint velocity command to Sawyer
		q_send = self.limb.joint_velocities();
		for j in range(0,7):
			q_send['right_j' + str(j)] = q[0,j];
		if VERBOSE:
			print('q_s = ' + str(q_send))
			print('delay = ' + str(rospy.get_time()-self.lastTime))
		self.lastTime = rospy.get_time();
		self.limb.set_joint_velocities(q_send)

if __name__ == '__main__':
	listener()