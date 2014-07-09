#!/usr/bin/env python
"""
================================================
DiffDriveSimPose.py -- Pioneer Simulator 2D Pose
================================================

Reads from the ODE Simulation pose information
"""

import sys
from numpy import *
import math

import lib.handlers.handlerTemplates as handlerTemplates

class DiffDriveSimPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data):
        """
        Pose Handler for Simulated Differential drive robot in ODE
        """
        self.simulator = shared_data['Simulator']

    def getPose(self, cached=False):
        """ Returns the most recent (x,y,theta) reading from the simulator """

        plist = self.simulator.get2DPose()
        while plist[2] > math.pi:
            plist[2] = plist[2] - 2*math.pi
        while plist[2] < -math.pi:
            plist[2] = plist[2] + 2*math.pi

        #print plist[2]*(180.0/math.pi)
        self.pose = array([plist[0],plist[1],plist[2]])
        return self.pose

