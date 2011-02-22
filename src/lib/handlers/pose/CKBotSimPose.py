#!/usr/bin/env python
"""
==========================================
CKBotSimPose.py -- CKBot Simulator 2D Pose
==========================================

Reads from the ODE Simulation pose information
"""

import sys
from playerc import *
from numpy import *
from socket import *
import math

class poseHandler:
    def __init__(self, proj, shared_data):
	self.simulator = shared_data['Simulator']

    def getPose(self, cached=False):
        """ Returns the most recent (x,y,theta) reading from the simulator """

	plist = self.simulator.get2DPose(0)  

	if self.simulator.config == "Snake":
	    plist[2] = plist[2] - math.pi/2

	self.pose = array([plist[0],plist[1],plist[2]])  
	return self.pose
    
