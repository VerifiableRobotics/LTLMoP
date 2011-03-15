#!/usr/bin/env python
"""
===================================================
CKBotInit.py -- CKBot Initialization Handler
===================================================
"""

import os, sys, time
from numpy import *
sys.path.append("lib/platforms/ckbot")
sys.path.append("../../Downloads/CKBot/trunk")
from simulator.ode.ckbot import CKBotLib
import CKBotRun
from threading import Thread, Lock, Event

class initHandler:
    def __init__(self, proj, calib=False):

		# Initiate the CKBot runtime thread.
		self.runtime = CKBotThread(os.path.join(proj.ltlmop_root,"lib/simulator/ode/ckbot/config/Snake.ckbot"))
		self.config = self.runtime.runtime.config
		self.runtime.start()

		# Instantiate the CKBot library
		self.lib = CKBotLib.CKBotLib()

    def getSharedData(self):
        # Return a dictionary of any objects that will need to be shared with
        # other handlers

        return {'Runtime': self.runtime, "Library": self.lib, "Config": self.config}

class CKBotThread(Thread):
    def __init__(self, robotfile):
        super(CKBotThread, self).__init__()
        self.robotfile = robotfile
        self.lock = Lock()
        self._stop = Event()
        self.runtime = CKBotRun.CKBotRun(self.robotfile)

    def run (self):
		# Run the robot.
		while 1:
			if self._stop.isSet():
				break
			else:
				self.lock.acquire()
				self.runtime.run_once()
				self.lock.release()

    def setGait(self, gait):
        self.lock.acquire()
        self.runtime.setGait(gait)
        self.lock.release()

    def stop (self):
        self._stop.set()
