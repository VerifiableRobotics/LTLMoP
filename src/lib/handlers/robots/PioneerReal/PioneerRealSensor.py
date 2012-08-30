#!/usr/bin/env python
"""
=========================================================
PioneerRealSensor.py - Real Pioneer Robot Sensor Handler
=========================================================
"""

class PioneerRealSensorHandler:
    def __init__(self, proj, shared_data):
        """
        Real Pioneer Robot Sensor handler
        """

        self.robocomm = shared_data['robocomm']


    ###################################
    ### Available sensor functions: ###
    ###################################
    def dynamicObstacles(self):
        """
        return a flag sent from Pioneer telling whether there is a dynamic obstacle nearby or not
        """
        #if self.robocomm.getSTOP() == True:
        #    print  "sensing dyanmic obstacles"
        return self.robocomm.getSTOP()

