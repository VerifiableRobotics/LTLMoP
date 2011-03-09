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

class actuatorHandler:

	def __init__(self, proj, shared_data):
		self.simulator = shared_data['Simulator']
		self.library = shared_data['Library']

	def setActuator(self, name, val):
		"""
		Sets CKBot configurations.
		"""	

		#if name=="slinky" and val==True:
		#	self.simulator.reconfigure("FoldOver")

		#elif name=="snake" and val==True:
		#	self.simulator.reconfigure("Snake")

		#elif name=="play_dead" and val==True:
		#	self.simulator.reconfigure("Plus3")

		#elif name=="hexapod" and val==True:
		#	self.simulator.reconfigure("Hexapod")

		#elif name=="cross" and val==True:
		#	self.simulator.reconfigure("Plus3")
	
		# Use library if we can		
		words = name.split("_and_")
		print "name is " + name
		print "desired words are " 
		print words
		libs = self.library
		libs.readLibe()
		config = libs.findGait(words)
		if (type(config) != type(None)) and (self.simulator.config != config) and (val==True):
			print "reconfiguring to:" + config
			self.simulator.reconfigure(config)

		# Make the default configuration Hexapod
		# After we're done with any gait, switch back to snake
		elif (name!="hexapod") and val==False:
			print "deconfiguring"
			self.simulator.reconfigure("Hexapod")

		print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))
