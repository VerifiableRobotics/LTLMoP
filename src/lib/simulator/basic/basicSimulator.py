#!/usr/bin/env python
"""
================================================================
basicSimulator.py -- A simple robot simulator provides pose by integrating given locomotion cmd
================================================================
"""
from numpy import array
from math import atan2
import time

class basicSimulator:
    def __init__(self, init_pose):
        """
        Initialization handler for pioneer ode simulated robot.
        
        init_pose is a 1-by-3 vector [x,y,orintation]
        """
        
        print "(Basic Simulator) Initializing Basic Simulator..."
        self.pose = array(init_pose) # current pose
        self.curVel = array([0.0,0.0]) # current velocity
        self.time = 0.0 # used to calculate time elapsed
        self.inertia = 1 # scale from 0 to 1, the bigger the scale the smaller the "inertia" is 
        
    def setVel(self,cmd):
        """
        Set the velocity of the robot, update the pose by simply inegrate the velocity
        
        cmd is a 1-by-2 vector represents the velocity
        """
        if self.time == 0.0:
            self.time = time.time()
        # update the velocity, assume the velocity takes times to change (to avoid local minimum)
        self.curVel = self.inertia*array(cmd)+(1-self.inertia)*self.curVel
        self.pose[0:2] = self.pose[0:2]+array(self.curVel)*(time.time()-self.time)
        self.time = time.time()
        # the orintation is kept the same (rad)
        # TODO: allows more robot models
    
    def getPose(self):
        """
        Returns the current pose of the robot
        """ 
        return self.pose   
