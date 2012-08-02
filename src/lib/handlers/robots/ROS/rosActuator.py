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
		self.message=None
		self.action=None

	#####################################
	### Available actuator functions: ###
	#####################################


	def simpleTopicAction(self, actuatorVal, topic='r_gripper_controller/command', messageType='Pr2GripperCommand', attr1Name='position', attr1Value='.2', attr2Name='max_effort', attr2Value='100', attr3Name='', attr3Value='', initial=False):
		"""
		Complete an action by publishing on a Topic	

		topic (str): The ROS topic with which to publish your action message (default='r_gripper_controller/command')
		messageType (str): The type of message to send on the topic (default='Pr2GripperCommand')
		attr1Name (str): The name of the 1st attribute of the message you want to set (default="position")
		attr1Value (str): The value to assign to the 1st attribute (default='.2')
		attr2Name (str): The name of the 2nd attribute of the message you want to set (default="max_effort")
		attr2Value (str): The value to assign to the 2nd attribute (default="100")
		attr3Name (str): The name of the 3rd attribute of the message you want to set (default="")
		attr3Value (str): The value to assign to the 3rd attribute (default="")
		"""
		if initial:
			#Open a publisher on the given topic with the given message type
			self.action=rospy.Publisher(topic,eval(messageType))
			#Generally you should initiate a node for each publisher, but
			#Locomotion has already done that, so there is now a global node
			#for ltlmop handlers, which this will publish on (you cannot 
			#create multiple nodes for a given process [ltlmop])
			#create a blank message with the given type
			self.message=eval(messageType+'()')
			#Set the attributes of the message
			if not attr1Name=='':
				setattr(self.message, attr1Name, eval(attr1Value))
			if not attr2Name=='':
				setattr(self.message, attr2Name, eval(attr2Value))
			if not attr3Name=='':
				setattr(self.message, attr3Name, eval(attr3Value))
		else:
			if int(actuatorVal)==1:
				#publish the message
				self.action.publish(self.message)
