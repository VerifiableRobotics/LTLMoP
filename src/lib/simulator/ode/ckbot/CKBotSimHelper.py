#!/usr/bin/env python

"""
This file contains all the helper functions common to all CKBot simulation variants.
"""

import ode, xode.parser
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math, time, copy, sys, os

from loadModules import *
from parseTextFiles import *
from matrixFunctions import *

def get2DPose(sim, num):
	"""
	Get the 2D Pose (x, y, yaw) of module 'num'
	"""

	# Find 2-D pose using module num's position and rotation matrix.
	# NOTE: Due to the definition of the axes, the (x,y) position is actually (x,-z)
	pose2d = [0,0,0]

	# X and Y values are the average of the lower and upper parts of the base module.
	pose2d[0] = 0.5*(sim.lowerjoint[num].getPosition()[0] + sim.upperjoint[num].getPosition()[0])
	pose2d[1] = -0.5*(sim.lowerjoint[num].getPosition()[2] + sim.upperjoint[num].getPosition()[2])

	# Find 2D Orientation using the rotation matrix and the x-direction unit vector.
	# Rotate the vector [1, 0, 0] using the rotation matrix and find the angle to the x-axis using the atan2 function (to cover all 4 quadrants)
	rot = sim.lowerjoint[num].getRotation()
	rotvec = rotate(sim.fwdvec,rot)		
	pose2d[2] = math.atan2(-rotvec[2],rotvec[0]) 

	return pose2d
	

def get2DPoseAndHeight(sim, num):
	"""
	Get the 2D Pose (x, y, yaw) of module 'num' and its height (z) above the ground.
	"""

	# The height above the ground, by simulator convention, is the Y value.
	pose2d = get2DPose(sim,num)
	height = 0.5*(sim.lowerjoint[num].getPosition()[1] + sim.upperjoint[num].getPosition()[1])
	pose2d.append(height)
	
	return pose2d
	

def reconfigure(sim, name):
	"""
	Removes the previous CKBot configuration and spawns a new one as specified in the function arguments.
	"""
	
	sim.counter = 0

	# Find the previous configuration's 2D orientation.
	angle = get2DPose(sim, 0)[2]
	#rot = genmatrix(angle,2)

	# Load the new robot data from the new file "name".ckbot.
	# Assumes we are running either from the same directory as simulator or from src/.
	print("==========\nReconfiguring: " + name + "\n==========")
	if sim.standalone == 1: 					# ran as main
		robotfile = "config/" + name + ".ckbot"
	else:										# ran from LTLMoP
		robotfile = "lib/simulator/ode/ckbot/config/" + name + ".ckbot"
	loadRobotData(sim, robotfile)

	# Update the position of the base module (always module 0) as the sum of the current configuration's pose
	# and the default base position matrix for a given configuration.
	relpos = [0.5*(sim.lowerjoint[0].getPosition()[0] + sim.upperjoint[0].getPosition()[0]),
		  0, 0.5*(sim.lowerjoint[0].getPosition()[2] + sim.upperjoint[0].getPosition()[2])]
	sim.basepos = [relpos[0]+sim.basepos[0], relpos[1]+sim.basepos[1]+0.1, relpos[2]+sim.basepos[2]]	

	# Update the orientation of the base module.
	newvec = rotate(sim.fwdvec,sim.baserot)
	newangle = math.atan2(-newvec[2],newvec[0])
	rot = genmatrix(angle-newangle,2)
	sim.baserot = multmatrix(rot,sim.baserot)

	# Reload the objects.
	loadModuleObjects(sim)

		
def setGait(sim,gait):
	"""
	Set the gait number for simulation
	"""
	sim.gait = gait

	
def rungait(sim, ref_angles = None):
	"""
	Runs the gait specified by the object variable "gait"
	"""
	
	time = sim.counter/sim.fps

	# If the gait is set to zero, use the gait creator reference angles to move.
	if sim.gait == 0 and ref_angles == None:
		for module_idx in range(len(sim.hinge)):
			sim.hinge[module_idx].setParam(ode.ParamVel, 0)
		
	elif ref_angles != None:
		for module_idx in range(len(sim.hinge)):
			true_ang = sim.hinge[module_idx].getAngle()
			ref_ang = ref_angles[module_idx]*(math.pi/180.0)/100.0
			servo_vel = sim.gain*(ref_ang-true_ang)
			sim.hinge[module_idx].setParam(ode.ParamVel, servo_vel)

	else:
		gait = sim.gaits[sim.gait - 1]
		gaittype = gait[0]
		
		# If the gait is of periodic type, read the rows in the following format.
		# ROW 1: Amplitudes
		# ROW 2: Frequencies
		# ROW 3: Phases
		if gaittype == "periodic":
			for module_idx in range(len(sim.hinge)):
				amplitude = gait[1][module_idx]
				frequency = gait[2][module_idx]
				phase = gait[3][module_idx]

				true_ang = sim.hinge[module_idx].getAngle()
				ref_ang = amplitude*math.sin(frequency*time + phase)
				servo_vel = sim.gain*(ref_ang-true_ang)
				sim.hinge[module_idx].setParam(ode.ParamVel, servo_vel)

		# If the gait is of fixed type, run the function "gaitangle" to interpolate between
		# reference hinge angles at the current time.
		elif gaittype == "fixed":
			for module_idx in range(len(sim.hinge)):
				true_ang = sim.hinge[module_idx].getAngle()	    
				ref_ang = gaitangle(sim,gait,time,module_idx)
				servo_vel = sim.gain*(ref_ang-true_ang)
				sim.hinge[module_idx].setParam(ode.ParamVel, servo_vel)


def gaitangle(sim, gait, time, module):
	"""
	Takes in a gait matrix and returns the reference angle at that point in time.
	"""

	nummoves = len(gait)-2
	gaittime = gait[1]
	singletime = float(gaittime)/(nummoves-1)

	timearray = []
	for i in range(0,nummoves):
		if i==0:
			timearray.append(0)
		else:
			timearray.append(timearray[i-1]+singletime)

	currenttime = (time%gaittime)/singletime
	globalref = gait[int(math.ceil(currenttime))+2][module]
	globalprev = gait[int(math.floor(currenttime))+2][module]

	if globalref==globalprev:
		# If the current time coincides with the time of a given gait angle direction, then there is no need to interpolate.
		localref = globalref
	else:
		# Linear interpolation step.
		interp = (currenttime*singletime - timearray[int(math.floor(currenttime))])/(timearray[int(math.ceil(currenttime))] - timearray[int(math.floor(currenttime))])
		localref = globalprev + interp*(globalref-globalprev)

	return localref

	
			
def set_periodic_gait_from_GA(sim, gene, gain, free_modules):
	"""
	Uses the GA state representation to set the robot's gait.
	Refer to "GA_Main.py" for information on this.
	"""
		
	amplitudes = []
	frequencies = []
	phases = []
	
	num_modules = len(sim.connM)
	counter = 0
	for i in range(num_modules):
		if i in free_modules:
			amplitudes.append( (gene[counter])*math.pi/180.0 )
			frequencies.append( 2*gene[counter+1] )
			phases.append( gene[counter+2]*36.0*math.pi/180.0 )
			counter = counter + 3
		else:
			amplitudes.append(0)
			frequencies.append(0)
			phases.append(0)
		
	sim.gaits = [["periodic", amplitudes, frequencies, phases]]
	sim.gait = 1
	sim.gain = gain

	return sim.gaits