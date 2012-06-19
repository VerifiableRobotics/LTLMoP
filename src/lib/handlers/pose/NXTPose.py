#!/usr/bin/env python
"""
=====================================================
NXTPose.py - Pose Handler for NXT with dead reckoning
=====================================================
"""

import sys, time
from numpy import *

class poseHandler:
    def __init__(self, proj, shared_data, differentialDrive=True):
        """
        Pose handler for the NXT with dead reckoning
        
        differentialDrive (bool): Whether the robot is differential Drive (default=True)
        """
        
        self.x = 0
        self.y = 0
        self.theta = 0
        self.dd=differentialDrive

    def getPose(self, cached=False):
        
        x=self.x
        y=self.y
        o=self.theta

        return array([x, y, o])
        
    def setPose(self, x, y, theta):
        self.x=x
        self.y=y
        self.theta=theta
        
