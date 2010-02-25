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

        self.actionValues = [{'name': 'radio',
                            'ON_cmdValue': [2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            'OFF_cmdValue': [2, 70, 0, 0, 0, 0, 0, 0, 0, 0]},
                            {'name': 'pick_up',
                            'ON_cmdValue': [2, 20, 0, 0, 0, 0, 0, 0, 0, 0],
                            'OFF_cmdValue': [2, 70, 0, 0, 0, 0, 0, 0, 0, 0]},
                            {'name': 'drop',
                            'ON_cmdValue': [2, 40, 0, 0, 0, 0, 0, 0, 0, 0],
                            'OFF_cmdValue': [2, 70, 0, 0, 0, 0, 0, 0, 0, 0]},
                            {'name': 'extinguish',
                            'ON_cmdValue': [3, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            'OFF_cmdValue': [3, 70, 0, 0, 0, 0, 0, 0, 0, 0]}]

    def setActuator(self, name, val):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (binary).
        """

        for value in self.actionValues:
            if name == value['name']:
                if int(val):
                    print "%s ON" % value['name']
                    cmdValue = value['ON_cmdValue']
                else:
                    print "%s OFF" % value['name']
                    cmdValue = value['OFF_cmdValue']

                cmd_data = pyarray.array('d')
                cmd_data.fromlist(cmdValue)
                self.gimbal_sock.sendto(cmd_data, (self.GIMBAL_HOST, self.GIMBAL_PORT))

        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

