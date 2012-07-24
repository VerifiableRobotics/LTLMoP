#!/usr/bin/env python
"""
=================================================
differentialDrive.py - Differential Drive Handler
=================================================

Converts a desired global velocity vector into translational and rotational rates for a differential-drive robot,
using feedback linearization.
"""

import math
from geometry_msgs.msg import Twist
#from gazebo.srv import *

class driveHandler:
    def __init__(self, proj, shared_data):
        try:
            self.loco = proj.h_instance['locomotionCommand']
            self.coordmap = proj.coordmap_lab2map
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
        #print "VEL:%f,%f" % tuple(self.coordmap([x, y]))
	twist = Twist()
	self.turning = False
        # Feedback linearization code:
        d = 0.3 # Distance from front axle to point we are abstracting to [m]
        vx= 0.09*x
	vy= 0.09*y
        w = (1/d)*(-math.sin(theta)*vx + math.cos(theta)*vy)
        v = math.cos(theta)*vx + math.sin(theta)*vy
        
        twist.linear.x = 0 
        twist.linear.y = 0 
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
        twist.angular.z = 0
	try:
	    if w!=0:	
                if  (not self.turning and abs(w) > math.pi/w):
                    self.turning = True
                    #print "not turning"
                    if (self.turning):
                        twist.angular.z = cmp (v,0)
                        #twist.angular.z = -5 * cmp(v,0) 
                        #print "turning"  
                else:
                    twist.linear.x = v 
                    twist.linear.y = w 
	    else:	
                if not self.turning:
                    self.turning = True
                    #print "not turning"
                    if (self.turning):
                        twist.angular.z = cmp (v,0)
                        #twist.angular.z = -5 * cmp(v,0) 
                        #print "turning"  
                else:
                    twist.linear.x = v 
                    twist.linear.y = w 
	except: 
	    twist.linear.x = v
            twist.linear.y = w

	print 'Twist: ',twist
        self.loco.sendCommand(twist)


