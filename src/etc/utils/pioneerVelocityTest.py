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

VEL2D_HOST = "10.0.0.182"
VEL2D_PORT = 13339

# Open up sockets
print '(LOCO) Connecting to Orca server...'
vel2d_sock = socket(AF_INET, SOCK_DGRAM)

print "(LOCO) OK! We've successfully connected."

cmd = [0.1, 0.1]
cmd_data = pyarray.array('d')
cmd_data.fromlist([cmd[0], 0, cmd[1]])
vel2d_sock.sendto(cmd_data, (VEL2D_HOST, VEL2D_PORT))

