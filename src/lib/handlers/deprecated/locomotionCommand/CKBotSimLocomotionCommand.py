#!/usr/bin/env python
"""
=========================================================================
CKBotSimLocomotionCommand.py - Simulated CKBot Locomotion Command Handler
=========================================================================
"""

import sys
from simulator.ode.ckbot import CKBotSimHelper

class locomotionCommandHandler:

	def __init__(self, proj, shared_data):

		self.simulator = shared_data['Simulator']

	def sendCommand(self, cmd):
		
		# Command the robot based on the gait given by the drive handler.
		CKBotSimHelper.setGait(self.simulator, cmd)
		self.simulator.run_once()

