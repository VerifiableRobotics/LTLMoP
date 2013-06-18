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
    def __init__(self, proj, shared_data,host,port):
        """
        Pose Handler for simulated pioneer ode robot.

        host (string): The host for communication with simulator, must match the one in UDPServer.py (default="localhost")
        port (int): The port for communication with simulator, must match the one in UDPServer.py (default=23456)
        """
        self.host = host
        self.port = port
                
    def getPose(self, cached=False):
              
        """ Returns the most recent (x,y,theta) reading from the simulator """

        time.sleep(0.05)
                
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)        
            sock.sendto("LTLMOP%ignore" + "\n", (self.host, self.port))
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
    
