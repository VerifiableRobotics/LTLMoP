#!/usr/bin/env python
"""
==========================================
CKBotSimPose.py -- CKBot Simulator 2D Pose
==========================================

Reads from the ODE Simulation pose information
"""

import sys
from numpy import *
from simulator.ode.ckbot import CKBotSimHelper
import math

import lib.handlers.handlerTemplates as handlerTemplates

class CKBotSimPoseHandler(handlerTemplates.PoseHandler):

    def __init__(self, proj, shared_data):
        """
        Pose Handler for simulated CKBot robot
        """
        self.simulator = shared_data['Simulator']

    def getPose(self, cached=False):
        """ Returns the most recent (x,y,theta) reading from the simulator """
        
        plist = CKBotSimHelper.get2DPose(self.simulator, 0)  
        self.pose = array([plist[0],plist[1],plist[2]])  

        return self.pose
