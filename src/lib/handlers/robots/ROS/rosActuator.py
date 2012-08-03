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
from trajectory_msgs.msg import *

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
			pass
		else:
			if int(actuatorVal)==1:
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
				#publish the message
				first=time.time()
				last=first
				while True:
					self.action.publish(self.message)
					last=time.time()
					if last-first>20:
						break

	def sendGoal(self, actuatorVal, topic='torso_controller/position_joint_action', messageType='SingleJointPosition',attrNames='position',attrVals='0.2',initial=False):
		"""
		Goals are ways of actuation when you have a destination in mind

		topic (str): The topic to publish the command on (default="torso_controller/position_joint_action")
		messageType (str): The base name of the message type to send (default="SingleJointPosition")
		attrNames (str): The attribute names delimited by the * symbol (default="position")
		attrVals (str): The attribute values delimited by the * symbol (default="0.2")
		"""
		if initial:
			pass
		else:
			if int(actuatorVal)==1:
				self.client = actionlib.SimpleActionClient(topic, eval(messageType+'Action'))
				self.client.wait_for_server()
	
				self.message=eval(messageType+'Goal()')
				attributes=attrNames.split('*')
				vals=()
				values = attrVals.split('*')
				for val in values:
					try:
						vals+=eval(val),
					except:
						vals+=val,
				for i in range(len(attributes)):
					setattr(self.message, attributes[i], vals[i])

				self.client.send_goal(self.message)
				self.client.wait_for_result()
