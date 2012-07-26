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
			#self.pub = rospy.Publisher('/gazebo/set_model_state', Twist)
			rospy.init_node('locomotionCommandHandler')
			self.lastcmd=Twist()
			self.pub.publish(self.lastcmd)
		except:
			print 'Problem setting up Locomotion Command Node'

	def sendCommand(self, cmd):
		#  Just set linear velocity to cmd[0] and cmd[1] for x and y velocities
		#  Then publish them to the base controller to move the pr2
		#  Currently multiplying it by 5 to try and make the pr2 move faster but does seem to have much affect
		#twist = Twist()
		#if cmd[0] < 0:
		#    twist.angular.z= cmd[0] 
		#    twist.linear.x = cmd[0]  
	    #if cmd [1] < 0:
		#    twist.angular.z= cmd[1]  
		#    twist.linear.y = cmd[1]  
		#else:	
		#    twist.linear.x = cmd[0] * 2
		#    twist.linear.y = cmd[1] * 2 

		#print "Twist",cmd[0],cmd[1]
		#print 'Twist',cmd
		self.lastcmd=cmd
		try:
			if not rospy.is_shutdown():
				self.pub.publish(cmd)
				rospy.loginfo('Twist Command: '+ str(cmd))
		except:
			print 'Error publishing Twist Command'

