#!/usr/bin/env python
"""
================================================
DiffDriveSimPose.py -- Pioneer Simulator 2D Pose
================================================

Reads from the ODE Simulation pose information
"""

import sys, socket, os, subprocess, time
from numpy import *
import math

class poseHandler:
    def __init__(self, proj, shared_data):
        """
        Initialize pose handler for PioneerODE
        """
        pass
                
    def getPose(self, cached=False):
              
        """ Returns the most recent (x,y,theta) reading from the simulator """

        time.sleep(0.05)
                
        try:
                HOST, PORT = "localhost", 23456
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)        
                sock.sendto("LTLMOP%ignore" + "\n", (HOST, PORT))
                received = sock.recv(1024)
        
                plist = eval(received) 
                while plist[2] > math.pi:
                        plist[2] = plist[2] - 2*math.pi
                while plist[2] < -math.pi:
                        plist[2] = plist[2] + 2*math.pi
                
                self.pose = array([plist[0],plist[1],plist[2]])  
        #print plist[2]*(180.0/math.pi)
        except:
                self.getPose()
                        
        return self.pose
    
