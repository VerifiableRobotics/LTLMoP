#!/usr/bin/env python
"""
=======================================================================
CKBotSimActuator.py - CKBotSimulator Actuator [Reconfiguration] Handler
=======================================================================

Does nothing more than print the actuator name and state; for testing purposes.
"""

import time, math, sys, os
from simulator.ode.ckbot import CKBotSimHelper

class actuatorHandler:

	def __init__(self, proj, shared_data):
		self.simulator = shared_data['Simulator']
		self.library = shared_data['Library']
		self.trueTraits = set([])

	def setActuator(self, name, val):
		"""
		Sets CKBot configurations.
		"""	
		
		print name + " is set to " + val
		
		if name[0]== "T" and val==True:
			self.trueTraits.add(name.lstrip("T_"))
		elif name[0] == "T" and val==False:
			self.trueTraits.discard(name.lstrip("T_"))

		print "Current trait list:"
		print self.trueTraits

		#if name=="slinky" and val==True:
		#	self.simulator.reconfigure("FoldOver")

		#elif name=="snake" and val==True:
		#	CKBotSimHelper.reconfigure(self.simulator,"Snake")

		#elif name=="play_dead" and val==True:
		#	self.simulator.reconfigure("Plus3")

		#elif name=="hexapod" and val==True:
		#	self.simulator.reconfigure("Hexapod")

		#elif name=="cross" and val==True:
		#	self.simulator.reconfigure("Plus3")
	
		libs = self.library
		libs.readLibe()

		config = libs.findGait(self.trueTraits)
		if (type(config) != type(None)) and (self.simulator.config != config):
			print "reconfiguring to:" + config
			CKBotSimHelper.reconfigure(self.simulator, config)

#		# Use library if actuator name starts with T
#		if name[0] == "T" and val==True:
#			words = name.split("_and_")
#			words[0] = words[0].lstrip("T_")
#			print "name is " + name
#			print "desired words are " 
#			print words
#			config = libs.findGait(words)
#			if (type(config) != type(None)) and (self.simulator.config != config) and (val==True):
#				print "reconfiguring to:" + config
#				self.simulator.reconfigure(config)
			# If no gait is found from traits library, then it will just continue with whatever config-gait it's in

		# Make the default configuration Hexapod
		# After we're done with any gait, switch back to default
		elif (type(config) == type(None)) and (self.simulator.config != "Snake"):
			print "deconfiguring"
			CKBotSimHelper.reconfigure(self.simulator, "Snake")

		

		print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))



#	def updateTrueTraits(self,trait,truth):
#		if truth == True:
#			self.trueTraits.add(trait)
#		elif truth == False:
#			self.trueTraits.discard(trait)
