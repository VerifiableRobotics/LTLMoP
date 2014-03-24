#!/usr/bin/env python
"""
=======================================
basicSimPose.py - 2D Pose provider for basicSimulator
=======================================
"""

import sys
from numpy import *

import lib.handlers.handlerTemplates as handlerTemplates

class BasicSimPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data):
        """
        Pose Handler for basic simulated robot
        """
        try:
            self.simulator = shared_data['BasicSimulator']
        except KeyError:
            print "(POSE) ERROR: Basic Simulator doesn't seem to be initialized!"
            sys.exit(-1)

        self.last_pose = None

    def getPose(self, cached=False):
        """ Returns the most recent (x,y,theta) reading from basic simulator """

        if not cached or self.last_pose is None:
            # Get updated information
            self.last_pose = self.simulator.getPose()
        return self.last_pose


