 #!/usr/bin/env python
"""
================================================================================
CSharpRobotLocomotionCommand.py - Pioneer/Segway Locomotion Command Handler
================================================================================
"""
import socket, sys, time


class locomotionCommandHandler:
    def __init__(self, proj, shared_data):
        try:
            self.robocomm = shared_data['robocomm']
        except KeyError, ValueError:
            print "(LOCO) ERROR: No RobotCommunicator set to key 'robocomm' in shared data from init."
            exit(-1)
    
    def sendCommand(self, cmd):
        # Command the robot based on the gait given by the drive handler.
        direction = (0.7*cmd[0],0.7*cmd[1])
        self.robocomm.sendDirection(direction)
