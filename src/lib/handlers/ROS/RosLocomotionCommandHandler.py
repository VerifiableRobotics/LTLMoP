#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo')

import rospy, math, subprocess, os, sys
from gazebo.srv import *

from geometry_msgs.msg import Twist
"""
==================================================================
rosLocomotionCommand.py - ros Locomotion Command Handler
==================================================================
"""

import lib.handlers.handlerTemplates as handlerTemplates

class RosLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
	def __init__(self, executor, shared_data, velocityTopic='/base_controller/command'):
		"""
		The ROS Locomotion Command Handler

		velocityTopic (str): This is the topic which handles the movement commands (default='/base_controller/command')
		"""
		try:
			#open a publisher for the base controller of the robot
			self.pub = rospy.Publisher(velocityTopic, Twist)
			# for the pr2, use /base_controller/command
			# the turtlebot takes /cmd_vel
		except:
			print 'Problem setting up Locomotion Command Node'

	def sendCommand(self, cmd):

		#Twist is the message type and consists of x,y,z linear velocities
		#and roll, pitch, yaw orientation velocities (x,y,z)
		twist=Twist()
		#Positive x is forward on robots in Gazebo
		twist.linear.x=cmd[0]*4
		#Positive z is upward on robots in Gazebo
		twist.linear.z=cmd[2]*4
		#Angluar z is yaw or rotation in the xy plane
		twist.angular.z=cmd[1]*1.5
		try:
			#Publish the command to the robot
			self.pub.publish(twist)
		except:
			print 'Error publishing Twist Command'

