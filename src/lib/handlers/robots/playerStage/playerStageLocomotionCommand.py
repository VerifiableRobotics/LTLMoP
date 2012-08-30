#!/usr/bin/env python
"""
================================================
playerLocomotionCommand.py - Player Pos2D Client
================================================

Sends velocity commands to player
"""

import sys

class locomotionCommandHandler:
    def __init__(self, proj, shared_data):
        """
        LocomotionCommand Handler for player stage robot1
        """
        
        try:
            self.p = shared_data['PlayerPos2D']
        except KeyError:
            print "(LOCO) ERROR: Player doesn't seem to be initialized!"
            sys.exit(-1)
        pass

    def sendCommand(self, cmd):
        """ Send command to player.  Arguments depend on robot model. """ 

        self.p.set_cmd_vel(cmd[0], cmd[1], 0, 1)        

