#!/usr/bin/env python
"""
Sends X,Y velocity commands to Orca
"""

class locomotionCommandHandler:
    def __init__(self, shared_data):
        try:
            self.sock = shared_data['Vel2DSock']
            self.VEL2D_HOST = shared_data['VEL2D_HOST']
            self.VEL2D_PORT = shared_data['VEL2D_PORT']
        except KeyError:
            print "(LOCO) ERROR: Orca connection doesn't seem to be initialized!"
            sys.exit(-1)

    def sendCommand(self, cmd):
        # We are expecting a [vt, vw] command
        cmd_data = array.array('d')
        cmd_data.fromlist([cmd[0], 0, cmd[1]])
        self.sock.sendto(cmd_data, (self.VEL2D_HOST, self.VEL2D_PORT))

