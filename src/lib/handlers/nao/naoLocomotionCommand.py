#!/usr/bin/env python
"""
==================================================================
naoLocomotionCommand.py - Nao Locomotion Command Handler
==================================================================

Send forward, side, and angular velocity commands to the Nao.
"""

class naoLocomotionCommandHandler:
    def __init__(self, proj, shared_data):
        """
        Locomotion Command handler for NAO robot.

        proj project obj
        """
            
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']
        self.movProxy = self.naoInitHandler.createProxy('ALMotion')

    def sendCommand(self, cmd):
        """    Send movement command to the Nao
        Uses built in function in Aldebaran toolkit
        
        Expects [vel_x,vel_y,vel_ang,freq_step]"""
        
        # Extract data
        vx = cmd[0]
        vy = cmd[1]
        w = cmd[2]
        f = cmd[3]
        
        # Call robot movement
        self.movProxy.setWalkTargetVelocity(vx,vy,w,f)
