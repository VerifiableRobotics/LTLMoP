#!/usr/bin/env python
"""
================================================
CKBotSimDrive.py - Simulated CKBot Drive Handler
================================================
"""
import math

class driveHandler:

    def __init__(self, proj, shared_data):
        # Get reference to locomotion command handler, since
        # we'll be passing commands right on to it
	
		self.gait = 0
		self.runtime = shared_data['Runtime']
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
		old_gait = self.gait

		gait = 0
		# Check for zero (or really low) speed commands to assign no gait.
		# Only do this for the initial configuration (currently set to Hexapod)
		if (x==0 and y==0) and (self.runtime.config=="Hexapod"):
			gait = 0			# Do not move

		else:	

			# For the 4-Module Snake or 25 module Hexapod
			# If there is a non-zero speed command, select among forwards, left or right turn gaits.
			if self.runtime.config=="Hexapod":
				if (rel_heading<math.pi/24 and rel_heading>-math.pi/24):    
					gait = 1			# Go Straight
				elif (rel_heading>math.pi-math.pi/24 or rel_heading<-math.pi+math.pi/24):
					gait = 4			# Back Up
				elif (rel_heading<-math.pi/24 and rel_heading>-math.pi/2) or (rel_heading > math.pi/2 and rel_heading <math.pi-math.pi/24):
					gait = 2			# Go Right
				elif (rel_heading>math.pi/24 and rel_heading<math.pi/2) or (rel_heading<-math.pi/2 and rel_heading>-math.pi+math.pi/24):
					gait = 3			# Go Left

			# For the 4-Module Snake or 25 module Hexapod
			# If there is a non-zero speed command, select among forwards, left or right turn gaits.
			elif self.runtime.config=="Snake":
				if (rel_heading<math.pi/24 and rel_heading>-math.pi/24):    
					gait = 1			# Go Straight
				elif (rel_heading<-math.pi/24):
					gait = 2			# Go Right
				elif (rel_heading>math.pi/24):
						gait = 3			# Go Left

			elif self.runtime.config=="Plus":
				if (rel_heading >= -math.pi/4 and rel_heading < math.pi/4):
					gait = 3 			#  Horizontal forward
				elif (rel_heading >= math.pi/4 and rel_heading < 3*math.pi/4):
					gait = 2			#  Vertical backward
				elif (rel_heading >= 3*math.pi/4 and rel_heading < -3*math.pi/4):
					gait = 4			#  Horizontal backward
				else:
					gait = 1			#  Vertical forward	

			elif self.runtime.config=="Plus3":
				gait = 5			# play dead

			elif self.runtime.config=="FoldOver":
				gait = 2

			elif self.runtime.config=="Slinky":
				gait = 2			# move forward

			elif self.runtime.config=="Splits":
				gait = 1			# do the splits over and over

			elif self.runtime.config=="Twist":
				gait = 1

			elif self.runtime.config=="Quadriped":
				gait = 1

			else:
				gait = 0

		# If there is gait switching, print a message.
		self.gait = gait
		#if (old_gait != gait):
			#print "Activating Gait No." + str(gait)

		self.loco.sendCommand(gait)

