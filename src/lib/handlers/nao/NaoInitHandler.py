#!/usr/bin/env python
"""
=================================================
naoInit.py - Nao Initialization Handler
=================================================

Initialize the proxies to access Nao modules
"""

import naoqi
from naoqi import ALProxy

import lib.handlers.handlerTemplates as handlerTemplates

class NaoInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor, ip='nao.local',port=9559):
        """
        Initialization handler for NAO robot.

        ip (string): The ip address of NAO (default="nao.local")
        port (int): The port of NAO (default=9559)
        """

        try:
            # Get connection settings from robot configuration file
            self.naoIP = ip
            self.naoPort = port


        except KeyError, ValueError:
            print "ERROR: Cannot find Nao connection settings ('NaoIP', 'NaoPort') in robot file."
            return None

    def createProxy(self, name):
        try:
            return ALProxy(name, self.naoIP, self.naoPort)
        except RuntimeError:
            print "ERROR: Cannot create %s proxy to Nao." % name
            print "ERROR: Make sure the Nao is turned on and connected to the network."
            return None

    def getSharedData(self):
        return {'NAO_INIT_HANDLER': self}
