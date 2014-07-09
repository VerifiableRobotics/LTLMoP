#!/usr/bin/env python
"""
=======================================
skeletonPose.py - Skeleton Pose Handler
=======================================
"""

import sys, time
from numpy import *

class poseHandler:
    def __init__(self, executor, shared_data):
        # TODO: Initialize any network connections that might be necessary.
        #
        # This is only called once at the beginning of execution.
        #
        # Hostnames/port numbers can be loaded as proj.robot_data['XXXX_PORT'][0]
        # if you stick them in the robot configuration file
        #
        # Note that if the same network socket needs to be shared with another handlers,
        # you should initialize it in an initializationHandler and stick a reference
        # to the socket in shared_data.

        pass

    def getPose(self):
        # TODO: Find out current pose from the localization system
        [pos_x, pos_y, theta] = [0, 0, 0]

        return array([pos_x, pos_y, theta])

