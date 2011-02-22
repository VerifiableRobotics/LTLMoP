#!/usr/bin/env python
"""
=======================================================================
CKBotSimActuator.py - CKBotSimulator Actuator [Reconfiguration] Handler
=======================================================================

Does nothing more than print the actuator name and state; for testing purposes.
"""

import time, math
import sys
import os
from lib.simulator.ode.ckbot.CKBotLib import CKBotLib
#sys.path.append('home/cornell/Desktop/ltlmop-google/robots/CKBot')
sys.path.append('./robots/CKBot')
#from CKBotLib import *

class actuatorHandler:

	def __init__(self, proj, shared_data):
		self.simulator = shared_data['Simulator']

	def setActuator(self, name, val):
		"""
		Sets CKBot configurations.
		"""
		
		words = name.split("_and_")
		print "name is " + name
		print "desired words are " 
		print words
		libs = CKBotLib()
		libs.readLibe()
		config = libs.findGait(words)
		if (type(config) != type(None)) and (self.simulator.config != config) and (val==True):
			self.simulator.reconfigure(config)

		# Make the default configuration Snake
		# After we're done with any gait, switch back to snake
		if name!="drop" and val==False:
			self.simulator.reconfigure("Snake")

		print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))
