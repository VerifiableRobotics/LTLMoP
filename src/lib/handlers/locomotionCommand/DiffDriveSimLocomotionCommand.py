#!/usr/bin/env python
"""
================================================================================
DiffDriveSimLocomotionCommand.py - Pioneer Simulation Locomotion Command Handler
================================================================================
"""

class locomotionCommandHandler:
    def __init__(self, proj, shared_data):

	self.simulator = shared_data['Simulator']

    def sendCommand(self, cmd):

        # Command the robot based on the gait given by the drive handler.
        
		v = 5.0*cmd[0]
		w = 5.0*cmd[1]
		self.simulator.setVW(v,w)
		self.simulator.run_once()

