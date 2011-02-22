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
	self.simulator = shared_data['Simulator']
        try:
            self.loco = proj.loco_handler
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
        # TODO: Given x and y velocities figure out which gait should be selected. For the case of the Snake robot, for example, we choose between left, right and forward gaits here.

	# First get the current pose of the robot.
	velangle = math.atan2(y,x) + math.pi/2
	rel_heading = theta-velangle
	rel_heading = math.atan2(math.tan(rel_heading),1)   # Convert to go from -pi to +pi
	old_gait = self.gait

	# Check for zero (or really low) speed commands to assign no gait.
	magvel = math.sqrt( math.pow(x,2) + math.pow(y,2) )
	if (magvel < 1e-3):
	    gait = 0			# Do not move

	# For the 4-Module Snake
	# If there is a non-zero speed command, select among forwards, left or right turn gaits.
	elif self.simulator.config=="Snake":
	    if old_gait!=2 or old_gait!=3:
		if (rel_heading<math.pi/24 and rel_heading>-math.pi/24):    
		    gait = 1			# Go Straight
		elif (rel_heading<-math.pi/24):
		    gait = 2			# Go Left
		elif (rel_heading>math.pi/24):
		    gait = 3			# Go Right
	    elif (rel_heading<0.05 and rel_heading>-0.05):
		gait = 1
	    else:
		gait = 0

	elif self.simulator.config=="Plus":
	    #if (rel_heading >= -math.pi/4 and rel_heading < math.pi/4):
		#gait = 3 			#  Horizontal forward
	    #elif (rel_heading >= math.pi/4 and rel_heading < 3*math.pi/4):
		#gait = 2			#  Vertical backward
	    #elif (rel_heading >= 3*math.pi/4 and rel_heading < -3*math.pi/4):
		#gait = 3			#  Horizontal backward
	    #else:
		#gait = 4			#  Vertical forward
	    gait = 1	

	# If there is gait switching, print a message.
	self.gait = gait
	#if (old_gait != gait):
	    #print "Activating Gait No." + str(gait)

        self.loco.sendCommand(gait)

