#!/usr/bin/env python
"""
=====================================================
NXTPose.py - Pose Handler for NXT with dead reckoning
=====================================================
"""

import sys, time
from numpy import *
from regions import *
import _pyvicon
from math import pi

class poseHandler:
    def __init__(self, proj, shared_data, differentialDrive=True, compareAgainstVicon=False):
        """
        Pose handler for the NXT with dead reckoning
        
        differentialDrive (bool): Whether the robot is differential Drive (default=True)
        compareAgainstVicon (bool): Whether Vicon data is to be compared during this simulation (default=False)
        """
        
        r = 0
        center = proj.rfi.regions[r].getCenter()
        self.x = center[0]
        self.y = center[1]
        self.theta = pi/2
        self.dd=differentialDrive
        print 'Current Pose: '+str(self.x)+','+str(self.y)+','+str(self.theta)
        
        self.compareAgainstVicon=compareAgainstVicon
        if (compareAgainstVicon):
            self.s = _pyvicon.ViconStreamer()
            self.s.connect("10.0.0.102",800)
            self.s.selectStreams(['Time', "NXT:NXT <t-X>","NXT:NXT <t-Y>","NXT:NXT <a-Z>"])
            self.s.startStreams()
            while self.s.getData() is None: pass

    def getPose(self, cached=False):
        
        x=self.x
        y=self.y
        o=self.theta

        return array([x, y, o])
        
    def setPose(self, x, y, theta):
        self.x=x
        self.y=y
        self.theta=theta
        
