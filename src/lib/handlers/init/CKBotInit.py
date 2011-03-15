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

class initHandler:
    def __init__(self, proj, calib=False):

		# Initiate the CKBot runtime environment.
		self.runtime = CKBotRun.CKBotRun(os.path.join(proj.ltlmop_root,"lib/simulator/ode/ckbot/config/Snake.ckbot"))
		self.runtime.gait = 2
		for i in range(5):
			self.runtime.run_once()

		# Instantiate the CKBot library
		self.lib = CKBotLib.CKBotLib()

    def getSharedData(self):
        # Return a dictionary of any objects that will need to be shared with
        # other handlers

        return {'Runtime': self.runtime, "Library": self.lib}
