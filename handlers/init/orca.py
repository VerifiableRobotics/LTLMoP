#!/usr/bin/env python
"""
Sets up network connections to communicate with Orca
"""

from socket import *
import sys

class initHandler:
    def __init__(self, project_root, project_basename, exp_cfg_data, rdf_data, fwd_coordmap, rfi, calib=False):
        ### Connect to Orca:

        try:
            self.VEL2D_HOST = rdf_data['OrcaVelocityHost'][0]
            self.VEL2D_PORT = int(rdf_data['OrcaVelocityPort'][0])
            self.POS2D_GROUP = rdf_data['OrcaPositionGroup'][0]
            self.POS2D_PORT = int(rdf_data['OrcaPositionPort'][0])
        except KeyError, ValueError:
            print "ERROR: Cannot find Orca network settings in robot file."
            sys.exit(0)

        # Open up sockets
        print '(INIT) Connecting to Orca server...'
        self.vel2d_sock = socket(AF_INET, SOCK_DGRAM)
        self.pos2d_sock = socket(AF_INET, SOCK_DGRAM)
        self.pos2d_sock.bind(('', POS2D_PORT))

        # Join group
        group_bin = inet_pton(AF_INET, POS2D_GROUP)
        mreq = group_bin + struct.pack('=I', INADDR_ANY)
        self.pos2d_sock.setsockopt(IPPROTO_IP, IP_ADD_MEMBERSHIP, mreq)

        print "OK! We've successfully connected."
        
    def getSharedData(self):
        return {'Vel2DSock': self.vel2d_sock, 
                'Pos2DSock': self.pos2d_sock,
                'VEL2D_HOST': self.VEL2D_HOST,
                'VEL2D_PORT': self.VEL2D_PORT}
