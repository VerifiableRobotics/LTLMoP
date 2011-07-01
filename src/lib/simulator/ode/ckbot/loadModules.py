#!/usr/bin/env python

"""
This file contains all the functions needed for PyODE to create a configuration of CKBot modules.
"""

import ode, xode.parser
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math, time, copy, sys

from matrixFunctions import *

def loadModuleObjects(sim):
	"""
	Spawns the CKBot configuration based on the first module's position and orientation.
	All joint angles originally set to zero (0).
	"""  

	# Wipe the original lists of bodies, joints, hinges, etc.
	# This step is necessary for reconfiguration, since these lists are non-empty.
	sim.lowerbody = []
	sim.lowerjoint = []
	sim.upperbody = []
	sim.upperjoint = []
	sim.upperM = []
	sim.lowerM = []	
	sim.hinge = []
	sim.fixed = []

	# Figure out the connectivity of each module.
	# Each element in the "conninfo" list will show information as follows:
	# [Parent Module, Child Module, Parent Port, Child Port]
	conninfo = []
	for idx in range(len(sim.connM)):
		for j in range(idx,len(sim.connM)):
			if (sim.connM[idx][j] > 0):
				conninfo.append([idx, j, sim.connM[idx][j], sim.connM[j][idx]])

	# Create the first module.
	if (sim.connM != []):

		# Find the module positions and rotations relative to the specified base position.
		lower_offset = rotate((0, -sim.cubesize*0.25, 0), sim.baserot)					
		upper_offset = rotate((0, sim.cubesize*0.25, 0), sim.baserot)													
		rot = sim.baserot

		# Spawn the module.
		create_module(sim, sim.basepos, lower_offset, upper_offset, rot)

	# Spawn subsequent modules.
	idx = 1
	while (conninfo != []):
		for info in conninfo:
			if (info[1] == idx):  # This is the next module (in order) to spawn.
				parent = info[0]
				child = info[1]
				parent_port = info[2]
				child_port = info[3]
				conninfo.remove(info)

				# Create a fixed joint based on the connection type.
				### First, keep track of the parent module's position.
				#print "Creating connection: \n Modules   %d - %d \n Ports     %d - %d \n" %(parent,child,parent_port,child_port)
				parent_low_pos = sim.lowerbody[parent].getPosition()
				parent_up_pos = sim.upperbody[parent].getPosition()
				parent_pos = (0.5*(parent_low_pos[0]+parent_up_pos[0]),
							  0.5*(parent_low_pos[1]+parent_up_pos[1]),
							  0.5*(parent_low_pos[2]+parent_up_pos[2]))

				# Create the new module, rotated and translated based on the connection type.
				# These connection strategies are hard-coded in the giant chunk of code below.

				if parent_port == 1:
					if child_port == 1:

						### CONNECTION 1-1 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, -sim.cubesize*0.75, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((0, -sim.cubesize*1.25, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi,3))
					
						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 3:

						### CONNECTION 1-3 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize*0.25, -sim.cubesize, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((-sim.cubesize*0.25, -sim.cubesize, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(genmatrix(math.pi,2),genmatrix(-math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 5:

						### CONNECTION 1-5 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize*0.25, -sim.cubesize, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize*0.25, -sim.cubesize, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi/2,3))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 7:

						### CONNECTION 1-7 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, -sim.cubesize*1.25, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((0, -sim.cubesize*0.75, 0),sim.lowerbody[parent].getRotation())												
						rot = sim.lowerbody[parent].getRotation()

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[child],sim.lowerbody[parent])

					else:
							print"Invalid Connection: %d - %d"%(parent_port,child_port)

				elif parent_port == 2:
					if child_port == 2:

						### CONNECTION 2-2 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, -sim.cubesize*0.75, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((0, -sim.cubesize*1.25, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(genmatrix(math.pi,2),genmatrix(math.pi,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 4:

						### CONNECTION 2-4 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*0.25, -sim.cubesize, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((sim.cubesize*0.25, -sim.cubesize, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(genmatrix(math.pi,2),genmatrix(math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 6:

						### CONNECTION 2-6 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*0.25, -sim.cubesize, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((sim.cubesize*0.25, -sim.cubesize, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(-math.pi/2,3))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 7:

						### CONNECTION 2-7 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, -sim.cubesize*1.25, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((0, -sim.cubesize*0.75, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(-math.pi/2,2))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[child],sim.lowerbody[parent])

					else:
							print"Invalid Connection: %d - %d"%(parent_port,child_port)
							
				elif parent_port == 3:
					if child_port == 1:

						### CONNECTION 3-1 ###
						# Find the module positions and rotations relative to t0.5*(lower_pos[0]+upper_pos[0]),he parent position.
						lower_offset = rotate((sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(-math.pi/2,3))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 3:

						### CONNECTION 3-3 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize, sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((sim.cubesize, -sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(genmatrix(math.pi,1),genmatrix(math.pi,2))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 5:

						### CONNECTION 3-5 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize, sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((sim.cubesize, -sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi,1))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[child],sim.lowerbody[parent])

					elif child_port == 7:

						### CONNECTION 3-7 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())						
						upper_offset = rotate((sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())												
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi/2,3))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[child],sim.lowerbody[parent])

					else:
							print"Invalid Connection: %d - %d"%(parent_port,child_port)
							
				elif parent_port == 4:
					if child_port == 2:

						### CONNECTION 4-2 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi,1),genmatrix(-math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 4:

						### CONNECTION 4-4 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize, -sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize, sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi,2))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 6:

						### CONNECTION 4-6 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize, -sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize, sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						rot = sim.lowerbody[parent].getRotation()

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 7:	

						### CONNECTION 4-7 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(-math.pi/2,2),genmatrix(math.pi/2,1))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.upperbody[child])

					else:
						print"Invalid Connection: %d - %d"%(parent_port,child_port)
																											
				elif parent_port == 5:
					if child_port == 1:

						### CONNECTION 5-1 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi,1),genmatrix(math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 3:

						### CONNECTION 5-3 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize, sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize, -sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi,1))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 5:

						### CONNECTION 5-5 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize, sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize, -sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi,1),genmatrix(math.pi,2))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 7:

						### CONNECTION 5-7 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi,1),genmatrix(-math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.upperbody[child])

					else:
							print"Invalid Connection: %d - %d"%(parent_port,child_port)
																					
				elif parent_port == 6:
					if child_port == 2:

						### CONNECTION 6-2 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi/2,3))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 4: 

						### CONNECTION 6-4 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize, -sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize, sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						rot = sim.lowerbody[parent].getRotation()

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 6: 

						### CONNECTION 6-6 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize, -sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize, sim.cubesize*0.25, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi,2))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 7: 

						### CONNECTION 6-7 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi/2,1),genmatrix(-math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.upperbody[child])

					else:
							print"Invalid Connection: %d - %d"%(parent_port,child_port)
																					
				elif parent_port == 7:
					if child_port == 1: 

						### CONNECTION 7-1 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, sim.cubesize*0.75, 0),sim.upperbody[parent].getRotation())
						upper_offset = rotate((0, sim.cubesize*1.25, 0),sim.upperbody[parent].getRotation())
						rot = sim.upperbody[parent].getRotation()

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[parent],sim.lowerbody[child])

					elif child_port == 2: 

						### CONNECTION 7-2 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, sim.cubesize*0.75, 0),sim.upperbody[parent].getRotation())
						upper_offset = rotate((0, sim.cubesize*1.25, 0),sim.upperbody[parent].getRotation())
						rot = multmatrix(sim.upperbody[parent].getRotation(),genmatrix(math.pi/2,2))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[parent],sim.lowerbody[child])

					elif child_port == 3: 

						### CONNECTION 7-3 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*0.25, sim.cubesize, 0),sim.upperbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize*0.25, sim.cubesize, 0),sim.upperbody[parent].getRotation())
						rot = multmatrix(sim.upperbody[parent].getRotation(),genmatrix(-math.pi/2,3))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[parent],sim.lowerbody[child])

					elif child_port == 4: 

						### CONNECTION 7-4 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, sim.cubesize, sim.cubesize*0.25),sim.upperbody[parent].getRotation())
						upper_offset = rotate((0, sim.cubesize, -sim.cubesize*0.25),sim.upperbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi/2,2),genmatrix(-math.pi/2,3))
						rot = multmatrix(sim.upperbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[parent],sim.lowerbody[child])

					elif child_port == 5: 

						### CONNECTION 7-5 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*0.25, sim.cubesize, 0),sim.upperbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize*0.25, sim.cubesize, 0),sim.upperbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi,2),genmatrix(math.pi/2,3))
						rot = multmatrix(sim.upperbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[parent],sim.lowerbody[child])

					elif child_port == 6: 

						### CONNECTION 7-6 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, sim.cubesize, sim.cubesize*0.25),sim.upperbody[parent].getRotation())
						upper_offset = rotate((0, sim.cubesize, -sim.cubesize*0.25),sim.upperbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi/2,2),genmatrix(math.pi/2,3))
						rot = multmatrix(sim.upperbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[parent],sim.lowerbody[child])

					elif child_port == 7: 

						### CONNECTION 7-7 ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, sim.cubesize*1.25, 0),sim.upperbody[parent].getRotation())
						upper_offset = rotate((0, sim.cubesize*0.75, 0),sim.upperbody[parent].getRotation())
						rot = multmatrix(sim.upperbody[parent].getRotation(),genmatrix(math.pi,3))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.upperbody[parent],sim.upperbody[child])

					else:
							print"Invalid Connection: %d - %d"%(parent_port,child_port)
							
				elif parent_port == 8:
					if child_port == 8:

						### CONNECTION 8-8 [aka 1-2 & 2-1] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, -sim.cubesize*0.75, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((0, -sim.cubesize*1.25, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi/2,2),genmatrix(math.pi,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 9:

						### CONNECTION 8-9 [aka 1-4 & 2-3] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, -sim.cubesize, sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((0, -sim.cubesize, -sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(-math.pi/2,2),genmatrix(math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 10:

						### CONNECTION 8-10 [aka 1-6 & 2-5] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((0, -sim.cubesize, sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((0, -sim.cubesize, -sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi/2,2),genmatrix(-math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					else:
							print"Invalid Connection: %d - %d"%(parent_port,child_port)

				elif parent_port == 9:
					if child_port == 8:

						### CONNECTION 9-8 [aka 3-2 & 4-1] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi/2,1),genmatrix(-math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 9:

						### CONNECTION 9-9 [aka 3-4 & 4-3] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize, 0, sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize, 0, -sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(-math.pi/2,1),genmatrix(math.pi,2))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					elif child_port == 10:

						### CONNECTION 9-10 [aka 3-6 & 4-5] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((sim.cubesize, 0, sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((sim.cubesize, 0, -sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(-math.pi/2,1))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])

					else:
							print"Invalid Connection: %d - %d"%(parent_port,child_port)

				elif parent_port == 10:
					if child_port == 8:

						### CONNECTION 10-8 [aka 6-1 & 5-2] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize*0.75, 0, 0),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize*1.25, 0, 0),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi/2,1),genmatrix(math.pi/2,3))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])	

					elif child_port == 9:

						### CONNECTION 10-9 [aka 6-3 & 5-4] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize, 0, -sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize, 0, sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						rot = multmatrix(sim.lowerbody[parent].getRotation(),genmatrix(math.pi/2,1))

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])	

					elif child_port == 10:

						### CONNECTION 10-10 [aka 6-5 & 5-6] ###
						# Find the module positions and rotations relative to the parent position.
						lower_offset = rotate((-sim.cubesize, 0, -sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						upper_offset = rotate((-sim.cubesize, 0, sim.cubesize*0.25),sim.lowerbody[parent].getRotation())
						rot = multmatrix(genmatrix(math.pi/2,1),genmatrix(math.pi,2))
						rot = multmatrix(sim.lowerbody[parent].getRotation(),rot)

						# Spawn the module and fix it to its parent.
						create_module(sim, parent_pos, lower_offset, upper_offset, rot)
						create_fixed_joint(sim, sim.lowerbody[parent],sim.lowerbody[child])	

					else:
							print("Invalid Connection %i - %i");
							
		# Now that we have spawned one more module and connected it, move on to the next one.
		idx = idx + 1	

	# WHEW, THE END OF ALL THAT FINALLY!
	# Build the Geoms and Joints arrays for rendering.
	geomlist = sim.lowerjoint
	geomlist.extend(sim.upperjoint)
	geomlist.append(sim.ground)
	sim._geoms = geomlist
	sim._joints = sim.fixed
	sim._joints.extend(sim.hinge)
	
	
def create_module(sim, parent_pos, lower_offset, upper_offset, rot):
	"""
	Spawns a CKBot module at the upper and lower body positions and specified rotation matrix.
	"""

	# Find the absolute positions of the module given the parent's position.
	lower_pos = (parent_pos[0]+lower_offset[0],
				 parent_pos[1]+lower_offset[1],
				 parent_pos[2]+lower_offset[2])
	upper_pos = (parent_pos[0]+upper_offset[0],
				 parent_pos[1]+upper_offset[1],
				 parent_pos[2]+upper_offset[2])
	hinge_pos = (0.5*(lower_pos[0]+upper_pos[0]),
				 0.5*(lower_pos[1]+upper_pos[1]),
				 0.5*(lower_pos[2]+upper_pos[2]))

	# Create the lower piece of the module.
	lowerbody = ode.Body(sim.world)
	lowerjoint = ode.GeomBox(space=sim.space, lengths=(sim.cubesize, sim.cubesize*0.5, sim.cubesize) )
	lowerjoint.setBody(lowerbody)
	lowerjoint.setPosition(lower_pos)
	lowerjoint.setRotation(rot)
	lowerM = ode.Mass()
	lowerM.setBox(sim.cubemass,sim.cubesize,sim.cubesize*0.5,sim.cubesize)
	lowerbody.setMass(lowerM)

	# Create the upper piece of the module.
	upperbody = ode.Body(sim.world)
	upperjoint = ode.GeomBox(space=sim.space, lengths=(sim.cubesize, sim.cubesize*0.5, sim.cubesize) )
	upperjoint.setBody(upperbody)
	upperjoint.setPosition(upper_pos)
	upperjoint.setRotation(rot)
	upperM = ode.Mass()
	upperM.setBox(sim.cubemass,sim.cubesize,sim.cubesize*0.5,sim.cubesize)
	upperbody.setMass(upperM)

	# Create the hinge of the module.
	hinge = ode.HingeJoint(sim.world)
	hinge.attach(lowerbody,upperbody)
	hinge.setAnchor(hinge_pos)
	hinge.setAxis(rotate((1,0,0),rot))
	hinge.setParam(ode.ParamLoStop,sim.hingeminangle)
	hinge.setParam(ode.ParamHiStop,sim.hingemaxangle)
	hinge.setParam(ode.ParamFMax,sim.hingemaxforce)

	# Append all these new pointers to the simulator class.
	sim.lowerbody.append(lowerbody)
	sim.lowerjoint.append(lowerjoint)
	sim.lowerM.append(lowerM)
	sim.upperbody.append(upperbody)
	sim.upperjoint.append(upperjoint)
	sim.upperM.append(upperM)
	sim.hinge.append(hinge)


def create_fixed_joint(sim, body1, body2=ode.environment):
	"""
	Create a fixed joint between two bodies.
	"""
	
	fixed = ode.FixedJoint(sim.world)
	fixed.attach(body1,body2)
	fixed.setFixed()
	sim.fixed.append(fixed)
