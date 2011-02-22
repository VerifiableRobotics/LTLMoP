#!/usr/bin/env python
"""
=================================================
naoInit.py - Nao Initialization Handler
=================================================

Initialize the proxies to access Nao modules
"""

import sys, time
import naoqi
from naoqi import ALProxy

class initHandler:
    def __init__(self, proj, calib=False):
        try:
            # Get connection settings from robot configuration file
            naoIP = proj.robot_data['NaoIP'][0]             # IP address (string)
            naoPort = int(proj.robot_data['NaoPort'][0])    # Port (number)
        except KeyError, ValueError:
            print "(INIT) ERROR: Cannot find Nao connection settings ('NaoIP', 'NaoPort') in robot file."
            exit(-1)
        
        try:
            # Create proxies to access modules
            self.movProxy = ALProxy('ALMotion',naoIP,naoPort)
            self.memProxy = ALProxy('ALMemory',naoIP,naoPort)
            self.ttsProxy = ALProxy('ALTextToSpeech',naoIP,naoPort)
            self.ledProxy = ALProxy('ALLeds',naoIP,naoPort)
        except RuntimeError:
            print "(INIT) ERROR: Cannot connect to one or more of module proxies."
            print "Make sure the Nao is turned on and connected to the network."
            exit(-1)
        
    def getSharedData(self):
        # Return dictionary of module proxies for other handlers to use
        return {'mov':self.movProxy,'mem':self.memProxy,'tts':self.ttsProxy,'led':self.ledProxy}
