#!/usr/bin/env python
"""
===================================================
rosActuator.py - Actuation Halder for ROS intefarce
===================================================

Control functions using ROS
"""

import time
import threading
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

class rosActuatorHandler:
	def __init__(self, proj, shared_data):
		"""
		Actuator Handler for ROS type applications
		"""
		self.rosInitHandler = shared_data['ROS_INIT_HANDLER']

		#rospy.init_node('actuationHandler')

	#####################################
	### Available actuator functions: ###
	#####################################


	def simpleAction(self, actuatorVal, service='r_gripper_controller/gripper_action', initial=False):
		"""
		Perform a service call to complete an action

		service (str): The ROS service location (default='r_gripper_controller/gripper_action')
		"""
		if initial:
			pass
		else:
			if actuatorVale==True:
				pass
