#!/usr/bin/env python
"""
=========================================================
PioneerRealSensor.py - Real Pioneer Robot Sensor Handler 
=========================================================
"""

class PioneerRealSensorHandler:
    def __init__(self, proj, shared_data,robot_type):
        """
        Real Pioneer Robot Sensor handler
        
        robot_type (int): Which robot is used for execution. pioneer is 1, ODE is 2 (default=1)
        """
        if robot_type not in [1,2]:
            robot_type = 1
        self.system = robot_type
        if self.system == 1:
            self.robocomm = shared_data['robocomm']
        

    ###################################
    ### Available sensor functions: ###
    ###################################
    def dynamicObstacles(self):
        """
        return a flag sent from Pioneer telling whether there is a dynamic obstacle nearby or not
        """
        if self.system == 1:
            return self.robocomm.getSTOP()
        else:
            return False
            
