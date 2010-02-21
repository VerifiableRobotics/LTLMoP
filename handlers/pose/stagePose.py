#!/usr/bin/env python
"""
=====================================================
handlers/pose/stagePose.py - 2D Pose Client for Stage
=====================================================

Reads from Stage's simulated GPS

Prerequisites:
  * Player Client Initialization Handler
  * Stage Initialization Handler
"""

import sys
from playerc import *
from numpy import *

class poseHandler:
    def __init__(self, proj, shared_data):
        try:
            self.c = shared_data['PlayerClient']
            self.p = shared_data['PlayerPos2D']
        except KeyError:
            print "(POSE) ERROR: Player client doesn't seem to be initialized!"
            sys.exit(-1)

    def getPose(self):
        """ Returns the most recent (x,y,theta) reading from Stage's GPS """

        # Get updated information
        if self.c.read() == None:
            print playerc_error_str()
            print "(POSE) ERROR: Client disconnected."
            sys.exit(-1)

        return array([self.p.px, self.p.py, self.p.pa])
    

