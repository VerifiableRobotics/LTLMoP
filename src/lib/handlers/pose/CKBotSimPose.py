#!/usr/bin/env python
"""
==========================================
CKBotSimPose.py -- CKBot Simulator 2D Pose
==========================================

Reads from the ODE Simulation pose information
"""

import sys
from numpy import *
import math

class poseHandler:
    def __init__(self, proj, shared_data):
	self.simulator = shared_data['Simulator']

    def getPose(self, cached=False):
        """ Returns the most recent (x,y,theta) reading from the simulator """

	plist = self.simulator.get2DPose(0)  

	self.pose = array([plist[0],plist[1],plist[2]])  
	return self.pose
    
