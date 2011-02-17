#!/usr/bin/env python
"""
==================================================================
naoLocomotionCommand.py - Nao Locomotion Command Handler
==================================================================

Send forward, side, and angular velocity commands to the Nao.
"""

import naoqi
from naoqi import ALProxy

class locomotionCommandHandler:
	def __init__(self, proj, shared_data):
		# Get proxy
		try:
			self.movProxy = shared_data['mov']
		except KeyError, ValueError:
			print "(LOCO) ERROR: No ALMotion proxy set to key 'mov' in initialization handler."
			exit(-1)

	def sendCommand(self, cmd):
		"""	Send movement command to the Nao
		Uses built in function in Aldebaran toolkit
		
		Expects [vel_x,vel_y,vel_ang,freq_step]"""
		
		# Extract data
		vx = cmd[0]
		vy = cmd[1]
		w = cmd[2]
		f = cmd[3]
		
		# Call robot movement
		self.movProxy.setWalkTargetVelocity(vx,vy,w,f)
