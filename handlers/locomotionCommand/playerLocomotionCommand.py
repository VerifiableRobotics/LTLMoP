#!/usr/bin/env python
"""
===========================================================================
handlers/locomotionCommand/playerLocomotionCommand.py - Player Pos2D Client
===========================================================================

Sends velocity commands to player
"""

class locomotionCommandHandler:
    def __init__(self, proj, shared_data):
        try:
            self.p = shared_data['PlayerPos2D']
        except KeyError:
            print "(LOCO) ERROR: Player doesn't seem to be initialized!"
            sys.exit(-1)
        pass

    def sendCommand(self, cmd):
        """ Send command to player.  Arguments depend on robot model. """ 

        self.p.set_cmd_vel(cmd[0], cmd[1], 0, 1)        

