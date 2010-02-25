#!/usr/bin/env python
"""
===========================================
gimbalActuator.py - Gimbal Actuator Handler
===========================================

Allows control of gimbal via Orca.
"""

from socket import *
from math import sin
import sys, struct, time
import array as pyarray

class actuatorHandler:
    def __init__(self, proj, shared_data):
        self.GIMBAL_PORT = 13337
        self.GIMBAL_HOST = "10.0.0.189"

        # Open up sockets
        print '(ACT) Connecting to Orca server...'

        self.gimbal_sock = socket(AF_INET, SOCK_DGRAM)

        print "(ACT) connected :)"

    def setActuator(self, name, val):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (binary).
        """

        if name == 'radio':
            if int(val):
                print "RADIO ON"
                ang = 0
            else:
                print "RADIO OFF"
                ang = 70

            cmd_data = pyarray.array('d')
            cmd_data.fromlist([2, ang, 0, 0, 0, 0, 0, 0, 0, 0])
            self.gimbal_sock.sendto(cmd_data, (self.GIMBAL_HOST, self.GIMBAL_PORT))

        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

