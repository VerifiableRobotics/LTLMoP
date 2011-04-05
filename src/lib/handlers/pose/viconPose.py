#!/usr/bin/env python
"""
============================================
viconPose.py - Pose Handler for Vicon System
============================================
"""

import sys, time
from numpy import *
import pyvicon

class poseHandler:
    def __init__(self, proj, shared_data):
        host = proj.lab_data['ViconHost'][0]    
        port = int(proj.lab_data['ViconPort'][0])

        self.s = pyvicon.ViconStreamer()
        self.s.connect(host,port)

        self.s.selectStreams(["Time", proj.robot_data['ViconName_X'][0], proj.robot_data['ViconName_Y'][0], proj.robot_data['ViconName_Theta'][0]])

        self.s.startStreams()

        # Wait for first data to come in
        while self.s.getData() is None: pass

    def getPose(self, cached=False):
        
        (t, x, y, o) = self.s.getData()
        (t, x, y, o) = [t/100, x/1000, y/1000, o]

        return array([x, y, o])

