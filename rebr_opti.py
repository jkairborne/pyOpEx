#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
import math
from math import sin, cos, pi, asin

import serial, string, math, time, calendar


# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped, PoseStamped


class PositionController(object):
	def __init__(self):
		self.model_name = 'Track1'

		# Containers
		self.optitrack_data = PoseStamped() # container to store incoming optitrack data

		# Pblishers and subscribers
		self.sub_optitrack_data = rospy.Subscriber('/optitrack/{0}/{0}'.format(self.model_name),PoseStamped, self.update_optitrack_data) # subscribe to incoming optitrack data

		# TIme tracking variables
		self.startTime = rospy.get_time(); # start time
		self.prevTime = self.startTime; # previous loop time

	def update_optitrack_data(self, actual_optitrack_data): # subscriber to obtain optitrack data
		self.optitrack_data = actual_optitrack_data

	def desired(self):
		# Actual position and orientation
		self.x = self.optitrack_data.pose.position.x
		self.y = self.optitrack_data.pose.position.y
		self.z = self.optitrack_data.pose.position.z
		quaternion = (self.optitrack_data.pose.orientation.x,self.optitrack_data.pose.orientation.y,self.optitrack_data.pose.orientation.z,self.optitrack_data.pose.orientation.w)
		(self.phi,self.theta,self.psi) = euler_from_quaternion(quaternion)
		#print "roll: %d, pitch:  %d, yaw:  %d" %(self.phi,self.theta,self.psi) 
		
		#SEND DATA OVER SERIAL

if __name__ == '__main__':
	rospy.init_node('rebr_opti') # initialize node
	obj = PositionController() # define class object
	rate = rospy.Rate(100) # loop rate

	while not rospy.is_shutdown():
		obj.desired() # call class function through object to publish command velocity 
		rate.sleep() # used to loop at desired rate
	rospy.spin()
