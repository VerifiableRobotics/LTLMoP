#!/usr/bin/env python
"""
================================================
CKBotDrive.py - CKBot Drive Handler
================================================
"""
import math, time

class driveHandler:

    def __init__(self, proj, shared_data):
        # Get reference to locomotion command handler, since
        # we'll be passing commands right on to it
	
		self.gait = 0
		self.runtime = shared_data['Runtime']
		self.config = shared_data['Config']
		self.pose_handler = proj.pose_handler

		try:
		    self.loco = proj.loco_handler
		except NameError:
		    print "(DRIVE) Locomotion Command Handler not found."
		    exit(-1)

    def setVelocity(self, x, y, theta=0):
        # TODO: Given x and y velocities figure out which gait should be selected. For the case of the Snake robot, for example, we choose between left, right and forward gaits here.

		# First get the current pose of the robot.
		pose = self.pose_handler.getPose()
		robotangle = pose[2]
		velangle = math.atan2(y,x)
		rel_heading = velangle-robotangle
		if (rel_heading > math.pi):
			rel_heading = rel_heading - 2*math.pi
		elif (rel_heading < -math.pi):
			rel_heading = rel_heading + 2*math.pi

		gait = 0
		# Check for zero (or really low) speed commands to assign no gait.
		# Only do this for the initial configuration (currently set to Hexapod)
		if (x==0 and y==0):
			gait = 0			# Do not move

		else:	

			# For the 4-Module Snake or 25 module Hexapod
			# If there is a non-zero speed command, select among forwards, left or right turn gaits.
			if self.config=="Snake":
				if (rel_heading<math.pi/24 and rel_heading>-math.pi/24):    
					gait = 1			# Go Straight
				elif (rel_heading<-math.pi/24):
					gait = 3			# Go Right
				elif (rel_heading>math.pi/24):
						gait = 2			# Go Left

			else:
				gait = 0

		self.loco.sendCommand(gait)

