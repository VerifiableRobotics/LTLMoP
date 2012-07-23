#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo')

import rospy, math, subprocess, os
from gazebo.srv import *

from geometry_msgs.msg import Twist
"""
==================================================================
gazeboLocomotionCommand.py - Gazebo Locomotion Command Handler
==================================================================
"""

class locomotionCommandHandler:
    def __init__(self, proj, shared_data):
       	self.pub = rospy.Publisher('/base_controller/command', Twist)
	rospy.init_node('locomotionCommand')

    def sendCommand(self, cmd):
	#  Just set linear velocity to cmd[0] and cmd[1] for x and y velocities
	#  Then publish them to the base controller to move the pr2
	#  Currently multiplying it by 5 to try and make the pr2 move faster but does seem to have much affect
#	twist = Twist()
#	if cmd[0] < 0:
#		twist.angular.z= cmd[0] * 5 
#		twist.linear.x = cmd[0]  
#        if cmd [1] < 0:
#		twist.angular.z= cmd[1] * 5 
#		twist.linear.y = cmd[1]  
#	else:	
#	twist.linear.x = cmd[0] * 2
#	twist.linear.y = cmd[1] * 2 

#	print "Twist",cmd[0],cmd[1]
	self.pub.publish(cmd)

