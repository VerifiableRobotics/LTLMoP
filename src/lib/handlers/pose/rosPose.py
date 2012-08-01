#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo')

import rospy, math
from gazebo.srv import *
from numpy import *
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


"""
=======================================
rosPose.py - ROS Interface Pose Handler
=======================================
"""

class poseHandler:
	def __init__(self, proj, shared_data):
		#GetModelState expects the arguments model_name and relative_entity_name
		#In this case it is pr2 and world respectively but can be changed for different robots and environments	
		self.model_name = 'pr2'
		self.relative_entity_name = 'world'
		self.last_pose = None
		self.pos_x = 0
		self.pos_y = 0
		self.or_x = 0
		self.or_y = 0
		self.or_z = 0
		self.or_w = 0

		self.shared_data=shared_data['ROS_INIT_HANDLER']

	def getPose(self,cached = False):
		if (not cached) or self.last_pose is None:
			#Ros service call to get model state
			#This returns a GetModelStateResponse, which contains data on pose and twist
			rospy.wait_for_service('/gazebo/get_model_state')
			try:
				gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
				resp = gms(self.model_name,self.relative_entity_name)
				self.pos_x = resp.pose.position.x
				self.pos_y = resp.pose.position.y
				self.or_x = resp.pose.orientation.x
				self.or_y = resp.pose.orientation.y
				self.or_z = resp.pose.orientation.z
				self.or_w = resp.pose.orientation.w
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			#  Use the tf module transforming quaternions to euler
			try:
				angles = euler_from_quaternion([self.or_x, self.or_y, self.or_z, self.or_w])
				self.theta = angles[2]	
				shared=self.shared_data
				self.last_pose = array([self.pos_x+shared.offset[0], self.pos_y+shared.offset[1], self.theta])
				#self.theta-=self.theta
				#while self.theta>pi: self.theta-=2*pi
				#while self.theta<-pi:self.theta+=2*pi
				#self.last_pose=array([real_pose[0]/shared.ratio+(shared.imgWidth/2),-real_pose[1]/shared.ratio+(shared.imgHeight/2),self.theta])
				#print self.last_pose
			except Exception:
				print 'Pose Broke', Exception
		return self.last_pose
