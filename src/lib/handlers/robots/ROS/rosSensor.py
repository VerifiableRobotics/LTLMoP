#!/usr/bin/env python
"""
====================================================
rosSensor.py - Sensor handler for the ROS interface
====================================================
"""

class rosSensorHandler:
    def __init__(self, proj, shared_data):
        self.rosInitHandler = shared_data['ROS_INIT_HANDLER']

    ###################################
    ### Available sensor functions: ###
    ###################################
