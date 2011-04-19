#!/usr/bin/env python
"""
=======================================================================
CKBotActuator.py - CKBot Actuator [Reconfiguration] Handler
=======================================================================

"""

import time, math
import sys
import os

class actuatorHandler:

	def __init__(self, proj, shared_data):
		self.runtime = shared_data['Runtime']
		self.library = shared_data['Library']
		self.config = shared_data['Config']
		self.traits = []

	def setActuator(self, name, val):
		"""
		Sets CKBot configurations.
		"""	
	
		# Use library if actuator name starts with T
		if name[0] == "T" and name[1] == "_" and val == True:
			name = name.lstrip('T_')
			
			self.traits.append(name)
			print "CURRENT TRAITS:"
			print self.traits

			libs = self.library
			libs.readLibe()
			config = libs.findGait(self.traits)
			if (type(config) != type(None)) and (self.config != config):
				print "Reconfiguring to: " + config
				# If no gait is found from traits library, then it will just continue with whatever config-gait it's in
				self.runtime.reconfigure(config)

		elif name[0] == "T" and name[1] == "_" and val==False:
			name = name.lstrip('T_')
			
			self.traits.remove(name)		
			print "CURRENT TRAITS:"
			print self.traits

			libs = self.library
			libs.readLibe()
			config = libs.findGait(self.traits)

			if (type(config) != type(None)) and (self.config != config):
				print "Reconfiguring to: " + config
				self.runtime.reconfigure(config)

			# If there are no gaits available then reconfigure to the default.
			elif (type(config) == type(None) or self.traits == ["hardware"]):
				self.runtime.reconfigure("Tee")
			#self.runtime.reconfigure("Tee")

		print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))
