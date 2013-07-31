#!/usr/bin/env python
"""
=======================================
CSharpPose.py - 2D Pose provider for CSharpRobot
=======================================
"""

import sys
from numpy import *

class poseHandler:
    def __init__(self, proj, shared_data,x=0.7,y=0.5,yaw=0):
        """
        Pose Handler for basic simulated robot
        x (float): initial pose x
        y (float): initial pose y 
        yaw (float): initial pose theta
        """
        self.CSharpCommunicator = proj.shared_data['robocomm']
        self.initPose=array([x,y,yaw])
        self.pose=self.initPose

    def getPose(self, cached=False):
        """ Returns the most recent (x,y,theta) reading from basic simulator """

        self.pose = self.CSharpCommunicator.getPose()
        if(self.pose!=None):
            #print array([self.pose.x,self.pose.y,self.pose.yaw])
            return array([self.pose.x,self.pose.y,self.pose.yaw])
        else:
            return self.initPose
    

