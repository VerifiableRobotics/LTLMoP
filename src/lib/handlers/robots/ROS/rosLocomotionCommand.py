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
			self.pub = rospy.Publisher('/base_controller/command', Twist)
			rospy.init_node('locomotionCommandHandler')
		except:
			print 'Problem setting up Locomotion Command Node'

	def sendCommand(self, cmd):

		twist=Twist()
		twist.linear.x=cmd[0]*4
		twist.angular.z=cmd[1]
		try:
			#if not rospy.is_shutdown():
			self.pub.publish(twist)
		except:
			print 'Error publishing Twist Command'

