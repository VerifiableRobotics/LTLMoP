#!/usr/bin/env python
"""
=======================================================================
CKBotSimActuator.py - CKBotSimulator Actuator [Reconfiguration] Handler
=======================================================================

Does nothing more than print the actuator name and state; for testing purposes.
"""

import time, math

class actuatorHandler:
    def __init__(self, proj, shared_data):
	self.simulator = shared_data['Simulator']

    def setActuator(self, name, val):
        """
        Sets CKBot configurations.
	"""

        if name == "snake" and val==True:
            if self.simulator.config != "Snake":
		self.simulator.reconfigure("Snake")

	if name == "cross" and val==True:
	    if self.simulator.config != "Cross":
		self.simulator.reconfigure("Plus")

        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

