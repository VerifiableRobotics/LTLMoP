#!/usr/bin/env python
"""
Reads from Stage's simulated GPS
"""

import sys
from playerc import *
from numpy import *

class poseHandler:
    def __init__(self, shared_data):
        try:
            self.c = shared_data['PlayerClient']
            self.p = shared_data['PlayerPos2D']
        except KeyError:
            print "(POSE) ERROR: Player doesn't seem to be initialized!"
            sys.exit(-1)
        
        self.lastPose = None # For caching

        # Test
        print "Initial pose: " + str(self.getPose())
            
    def getPose(self):
        # Get updated information
        if self.c.read() == None:
            print playerc_error_str()
            print "(POSE) ERROR: Client disconnected."
            sys.exit(-1)

        self.lastPose = array([self.p.px, self.p.py, self.p.pa])

        return self.lastPose
    
    def getLastPose(self):
        return self.lastPose

