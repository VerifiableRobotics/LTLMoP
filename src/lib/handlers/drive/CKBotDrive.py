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

			# If there is a non-zero speed command, select among forwards, left or right turn gaits.
			if self.config=="Snake":

				# If you were originally going straight, keep going straight until you deviate by a larger angle.
				if self.gait == 1:
					if (rel_heading<-math.pi/6):
						gait = 3			# Go Right
					elif (rel_heading>math.pi/6):
						gait = 2			# Go Left
					else:
						gait = 1			# Go Straight

				# Otherwise, if you were not originally going straight, only select to go straight if within a smaller angle.
				else:
					if (rel_heading<-math.pi/12):
						gait = 3			# Go Right
					elif (rel_heading>math.pi/12):
						gait = 2			# Go Left
					else:
						gait = 1			# Go Straight

			else:
				gait = 0

		self.gait = gait
		self.loco.sendCommand(gait)

