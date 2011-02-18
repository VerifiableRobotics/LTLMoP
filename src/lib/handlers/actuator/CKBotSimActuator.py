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
sys.path.append('./lib/simulator/ode/ckbot/config/')
sys.path.append('./lib/simulator/ode/ckbot/')
from CKBotLib import CKBotLib

class actuatorHandler:

	def __init__(self, proj, shared_data):
		self.simulator = shared_data['Simulator']

	def setActuator(self, name, val):
		"""
		Sets CKBot configurations.
		"""
	
		if name=="slinky" and val==True:
			self.simulator.reconfigure("FoldOver")

		elif name=="snake" and val==True:
			self.simulator.reconfigure("Snake")

		elif name=="play_dead" and val==True:
			self.simulator.reconfigure("Plus3")

		elif name=="hexapod" and val==True:
			self.simulator.reconfigure("Hexapod")
	
		# Use library if we can		
		words = name.split("_and_")
		#print "name is " + name
		#print "desired words are " 
		#print words
		libs = CKBotLib()
		libs.readLibe()
		config = libs.findGait(words)
		if (type(config) != type(None)) and (self.simulator.config != config) and (val==True):
			self.simulator.reconfigure(config)

		# Make the default configuration Hexapod
		# After we're done with any gait, switch back to snake
		elif name!="hoard" and val==False:
			self.simulator.reconfigure("Hexapod")

		print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))
