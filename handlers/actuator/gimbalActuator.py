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
import threading

class actuatorHandler:
    def __init__(self, proj, shared_data):
        self.GIMBAL_PORT = 13337
        self.GIMBAL_HOST = "10.0.0.189"

        # Open up sockets
        print '(ACT) Connecting to Orca server...'

        self.gimbal_sock = socket(AF_INET, SOCK_DGRAM)

        print "(ACT) connected :)"

        self.command = {'radio': {'ON': [2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    'OFF': [2, 70, 0, 0, 0, 0, 0, 0, 0, 0]},
                        'pick_up': {'ON': [2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    'OFF': [2, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
                        'drop': {'ON': [2, 70, 0, 0, 0, 0, 0, 0, 0, 0],
                                 'OFF': [2, 70, 0, 0, 0, 0, 0, 0, 0, 0]},
                        'extinguish': {'ON': [3, 70, 0, 0, 0, 0, 0, 0, 0, 0],
                                       'OFF': [3, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}

        # Reset gimbal to down-front
        self.sendCommand([2, 70, 0, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.1)
        self.sendCommand([3, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.doWiggle = False
        wiggleThread = threading.Thread(target = self.wiggle)
        wiggleThread.start()
    
    def wiggle(self):
        state = False
        while 1:
            if self.doWiggle:
                if state:
                    self.sendCommand(self.command['extinguish']['ON']) 
                else:
                    self.sendCommand(self.command['extinguish']['OFF']) 
                state = not state
            time.sleep(0.5)
            

    def sendCommand(self, cmd):
        cmd_data = pyarray.array('d')
        cmd_data.fromlist(cmd)
        self.gimbal_sock.sendto(cmd_data, (self.GIMBAL_HOST, self.GIMBAL_PORT))

    def setActuator(self, name, val):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (binary).
        """

        if name not in self.command:
            print "(ACT) Unknown actuator '%s'!" % name
            return

        if name == "pick_up" and not int(val):
            return

        # Special case for radio wiggle
        if name == "radio":
            if int(val):
                self.doWiggle = True
            else:
                self.doWiggle = False
                self.sendCommand(self.command['extinguish']['OFF']) 
            return

        if int(val):
            self.sendCommand(self.command[name]['ON'])
        else:
            self.sendCommand(self.command[name]['OFF'])

        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

