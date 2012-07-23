#!/usr/bin/env python
"""
===================================================
rosActuator.py - Actuation Halder for ROS intefarce
===================================================

Control functions using ROS
"""

import time
import threading

class rosActuatorHandler:
    def __init__(self, proj, shared_data):
        """

        """
        self.rosInitHandler = shared_data['ROS_INIT_HANDLER']


    #####################################
    ### Available actuator functions: ###
    #####################################

