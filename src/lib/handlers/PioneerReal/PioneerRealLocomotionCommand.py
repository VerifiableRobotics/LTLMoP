#!/usr/bin/env python
# -*- coding: cp1252 -*-
"""
================================================================================
PioneerRealLocomotionCommand.py - Real Pioneer Locomotion Command Handler
================================================================================
"""
import sys
class PioneerRealLocomotionCommandHandler:
    def __init__(self, proj, shared_data,scaleV,scaleW):
        """
        LocomotionCommand Handler for pioneer real robot.

        scaleV (float): The speed multiplier for V (default=1.0,min=1.0,max=2.0)
        scaleW (float): The speed multiplier for W (default=1.0,min=1.0,max=2.0)
        """

        try:
            self.robocomm = shared_data['robocomm']
        except KeyError, ValueError:
            print "(LOCO) ERROR: No RobotCommunicator set to key 'robocomm' in shared data from init."
            exit(-1)

        self.scaleV = scaleV
        self.scaleW = scaleW

    def sendCommand(self, cmd):
        """
        Command the robot based on the gait given by the drive handler.
        """

        direction = (cmd[0]*self.scaleV,self.scaleW*cmd[1])   # self.scaleV = 1, self.scaleW = 1.5

        self.robocomm.sendDirection(direction)
