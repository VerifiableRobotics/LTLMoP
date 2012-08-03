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

class locomotionCommandHandler:
	def __init__(self, proj, shared_data):
		try:
			#open a publisher for the base controller of the robot
			self.pub = rospy.Publisher('/base_controller/command', Twist)
			#The following is the global node for ltlmop handlers
			#rospy.init_node('LTLMoPHandlers')
		except:
			print 'Problem setting up Locomotion Command Node'

	def sendCommand(self, cmd):

		#Twist is the message type and consists of x,y,z linear velocities
		#and roll, pitch, yaw orientation velocities (x,y,z)
		twist=Twist()
		#Positive x is forward on robots in Gazebo
		twist.linear.x=cmd[0]*4
		#Angluar z is yaw or rotation in the xy plane
		twist.angular.z=cmd[1]
		try:
			#Publish the command to the robot
			self.pub.publish(twist)
		except:
			print 'Error publishing Twist Command'

