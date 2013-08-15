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
        # This option sends global velocity commands (linear and angular), aka a direction.
        direction = (0.7*cmd[0],0.7*cmd[1])
        self.robocomm.sendDirection(direction)

    def sendTargetPoint(self, cmd):
        # This option sends a waypoint (pt) to the robot. Path following is performed on the robot side.
        pt = (cmd[0,0],cmd[1,0])
        self.robocomm.sendDirection(pt)