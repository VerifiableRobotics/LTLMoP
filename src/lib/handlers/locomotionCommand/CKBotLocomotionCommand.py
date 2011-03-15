#!/usr/bin/env python
"""
=========================================================================
CKBotSimLocomotionCommand.py - Simulated CKBot Locomotion Command Handler
=========================================================================
"""

class locomotionCommandHandler:
    def __init__(self, proj, shared_data):

	self.runtime = shared_data['Runtime']

    def sendCommand(self, cmd):

        # Command the robot based on the gait given by the drive handler.
	    self.runtime.setGait(cmd)

