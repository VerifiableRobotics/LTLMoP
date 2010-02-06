#!/usr/bin/env python
"""
Sends X,Y velocity commands to player
"""

class locomotionCommandHandler:
    def __init__(self, shared_data):
        try:
            self.p = shared_data['PlayerPos2D']
        except KeyError:
            print "(POSE) ERROR: Player doesn't seem to be initialized!"
            sys.exit(-1)
        pass

    def sendCommand(self, cmd):
        self.p.set_cmd_vel(cmd[0], cmd[1], 0, 1)        

