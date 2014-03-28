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

import lib.handlers.handlerTemplates as handlerTemplates

class NXTInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor, brick='NXT', brickMAC='00:16:53:14:1B:33'):
        """
        Initialization handler for NXT robots.  If you are unsure of the mac address, please pass brickMac='none'.
        Your NXT Device will be located.

        brick (str): Name of brick to search for
        brickMAC (str): The MAC address of the nxt brick (default='00:16:53:14:1B:33')
        """

        if isinstance(brick, basestring):
            if brickMAC!='none': #check to see if user has given a mac address for the brick
                try:
                    brick = bluesock.BlueSock('00:16:53:14:1B:33').connect()  #attempt a connect with the mac address
                except:
                    print "Can't find given mac, searching for any brick"
            else:
                brick = find_one_brick(name=brick) #attempt to find an NXT device if no mac address is given
    
        self.brick = brick

    def getSharedData(self):
        return {'NXT_INIT_HANDLER': self}
