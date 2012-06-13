#!/usr/bin/env python
"""
========================================================
nxtInit.py - LEGO Mindstorms NXT Initialization Handler
========================================================

Initialize the bluetooth to access NXT modules
"""

from nxt import bluesock
from nxt.brick import Brick
from nxt.locator import find_one_brick
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C

class NXTInitHandler:
    def __init__(self, proj, brick='NXT', brickMAC='00:16:53:14:1B:33'):
        """
        Initialization handler for NXT robots.
        
        brickMAC (str): The MAC address of the nxt brick (default='00:16:53:14:1B:33')
        """

        if isinstance(brick, basestring):
            if brickMAC!='none':
                try:
                    brick = bluesock.BlueSock('00:16:53:14:1B:33').connect()
                except:
                    print "Can't find given mac, searching for any brick"
            else:
                brick = find_one_brick(name=brick)
    
        self.brick = brick

    def getSharedData(self):
        return {'NXT_INIT_HANDLER': self}
