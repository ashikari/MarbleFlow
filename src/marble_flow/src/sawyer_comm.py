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
		self.P_gain = 2e-3;
		self.I_gain = 0;
		self.D_gain = 0;
		self.int_err_x = 0;
		self.int_err_y = 0;
		self.x_bnd = [-0.27,0.38]
		self.y_bnd = [0.35,0.85]
		self.last_err_x = 0;
		self.last_err_y = 0;
		
		# Initialize Sawyer
		rospy.init_node('sawyer_comm', anonymous=True)
		self.limb = intera_interface.Limb('right')
		self.kin = sawyer_kinematics('right')

		joint_positions = go_to.joint_angle_arg(joint_angles = [math.pi/2, -math.pi/3, 0, 2*math.pi/3, 0, -math.pi/3, 0.57], speed_ratio = .225, accel_ratio = 0.1)
		go_to.joint_angles(joint_positions)
		self.start_pose = self.limb.endpoint_pose();
		if VERBOSE:
			self.lastTime = rospy.get_time()
		
		'''
		# Debug: move in a square
		s = 0.1
		self.limb.set_command_timeout(2)
		rate = rospy.Rate(100)
		x = np.array([s,0,0,0,0,0]);
		while self.limb.endpoint_pose()['position'][0]<0.25:
			self.set_endpoint_velocity(x)
			rate.sleep()
		x = np.array([0,-s,0,0,0,0]);
		while self.limb.endpoint_pose()['position'][1]>0.46:
			self.set_endpoint_velocity(x)
			rate.sleep()
		x = np.array([-s,0,0,0,0,0]);
		while self.limb.endpoint_pose()['position'][0]>-0.1658:
			self.set_endpoint_velocity(x)
			rate.sleep()
		x = np.array([0,s,0,0,0,0]);
		while self.limb.endpoint_pose()['position'][1]<0.7206:
			self.set_endpoint_velocity(x)
			rate.sleep()
		x = np.array([0,0,0,0,0,0]);
		self.set_endpoint_velocity(x)
		'''
	
		# Subscribing topics
		rospy.Subscriber('/ball_error', Int32MultiArray, callback=self.rx_pos_error, queue_size=1)

	def set_endpoint_velocity(self, x):
		q = np.matmul(self.kin.jacobian_pseudo_inverse(),x)
		q_send = self.limb.joint_velocities();
		for j in range(0,7):
			q_send['right_j' + str(j)] = q[0,j];
		self.limb.set_joint_velocities(q_send)

	def rx_pos_error(self, msg):
		self.limb.set_command_timeout(1)
		# Extract message contents
		if VERBOSE:
			print('\nI received: ' + str(msg.data[0]) + ',' + str(msg.data[1]))
		err_x = -msg.data[0];
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

		# TODO: Correct for drift in z position and endpoint orientation
		'''
		this_pose = self.limb.endpoint_pose();
		dx = np.array([0, 0, start_pose['position'][2]-self.limb.endpoint_pose()['position'][2], start_pose['orientation']
		ctrl+= 0.5*dx
		'''

		# Calculate joint velocities from inverse kinematics
		inv_J = self.kin.jacobian_pseudo_inverse()
		if (la.norm(inv_J, 'fro')>90):
			print('NEAR SINGULAR')
			rospy.signal_shutdown('Quit')
		endpoint_pos = self.limb.endpoint_pose()['position'];
		if (endpoint_pos[0]>self.x_bnd[1] or endpoint_pos[0]<self.x_bnd[0] or endpoint_pos[1]>self.y_bnd[1] or endpoint_pos[1]<self.y_bnd[0]):
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