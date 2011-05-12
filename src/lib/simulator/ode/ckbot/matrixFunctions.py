#!/usr/bin/env python

"""
This file is a mini linear algebra library for CKBot simulation.
"""

import ode, xode.parser
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math, time, copy, sys


def rotate(vec,rot):
	"""
	Rotates vector 'vec' using matrix 'mat'
	"""

	item1 = rot[0]*vec[0] + rot[1]*vec[1] + rot[2]*vec[2]
	item2 = rot[3]*vec[0] + rot[4]*vec[1] + rot[5]*vec[2]
	item3 = rot[6]*vec[0] + rot[7]*vec[1] + rot[8]*vec[2]
	newvec = [item1, item2, item3]

	#for i in range(0,3):
	#	if newvec[i]<1e-5 and newvec[i]>-1e-5:
	#		newvec[i] = 0

	return tuple(newvec)

	
def genmatrix(angle,axis):
	"""
	Generates a rotation matrix about the specified axis
	"""

	if (axis==1):	# X-AXIS
		rot = (1, 0, 0, 0, math.cos(angle), -math.sin(angle), 0, math.sin(angle), math.cos(angle))
	elif (axis==2):	# Y-AXIS
		rot = (math.cos(angle), 0, math.sin(angle), 0, 1, 0, -math.sin(angle), 0, math.cos(angle))
	elif (axis==3): # Z-AXIS
		rot = (math.cos(angle), -math.sin(angle), 0, math.sin(angle), math.cos(angle), 0, 0 , 0, 1)

	return rot


def multmatrix(M1,M2):
	"""
	Multiples two matrices, where each is defined as a 9-element list
	"""

	M = ( M1[0]*M2[0]+M1[1]*M2[3]+M1[2]*M2[6],
		  M1[0]*M2[1]+M1[1]*M2[4]+M1[2]*M2[7],
		  M1[0]*M2[2]+M1[1]*M2[5]+M1[2]*M2[8],
		  M1[3]*M2[0]+M1[4]*M2[3]+M1[5]*M2[6],
		  M1[3]*M2[1]+M1[4]*M2[4]+M1[5]*M2[7],
		  M1[3]*M2[2]+M1[4]*M2[5]+M1[5]*M2[8],
		  M1[6]*M2[0]+M1[7]*M2[3]+M1[8]*M2[6],
		  M1[6]*M2[1]+M1[7]*M2[4]+M1[8]*M2[7],
		  M1[6]*M2[2]+M1[7]*M2[5]+M1[8]*M2[8] )
		  
	return M
