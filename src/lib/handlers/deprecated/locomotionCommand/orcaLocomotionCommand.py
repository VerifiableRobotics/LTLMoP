#!/usr/bin/env python
"""
================================================
orcaLocomotionCommand.py - Orca Velocity Handler
================================================

Sends Vt, Vw velocity commands to Orca
"""
import sys
import array as pyarray  # So as not to conflict with numpy's array
from socket import *
from numpy import *

class locomotionCommandHandler:
    def __init__(self, proj, shared_data):
        """
        Opens socket and connects to Orca velocity server.

        Hostnames and ports are read in from the robot description file.
        """

        ### Connect to Orca:

        try:
            self.VEL2D_HOST = proj.robot_data['OrcaVelocityHost'][0]
            self.VEL2D_PORT = int(proj.robot_data['OrcaVelocityPort'][0])
        except KeyError, ValueError:
            print "(LOCO) ERROR: Cannot find Orca network settings ('OrcaVelocityHost', 'OrcaVelocityPort') in robot file."
            sys.exit(-1)

        # Open up sockets
        print '(LOCO) Connecting to Orca server...'
        self.vel2d_sock = socket(AF_INET, SOCK_DGRAM)

        print "(LOCO) OK! We've successfully connected."

    def sendCommand(self, cmd):
        """ Send a command to the Orca Velocy server.

            Expects [Vt, Vw].
        """

        cmd_data = pyarray.array('d')
        cmd_data.fromlist([cmd[0], 0, cmd[1]])
        self.vel2d_sock.sendto(cmd_data, (self.VEL2D_HOST, self.VEL2D_PORT))

