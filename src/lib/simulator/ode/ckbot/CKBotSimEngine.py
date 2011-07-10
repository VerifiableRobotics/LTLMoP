#!/usr/bin/env python

import ode, xode.parser
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math, time, copy, sys

from loadModules import *
from parseTextFiles import *
from matrixFunctions import *
from CKBotSimHelper import *


class CKBotSim:
	"""
	CKBot Simulator Class
	"""

	def __init__(self, robotfile,obstaclefile=None,regionfile=None,region_calib=None,startingpose=None,heightmap=None):
		"""
		Initialize the simulator.
		"""

		self.fps = 30.0
		
		# If regionfile=0, render the ground as default solid green terrain.
		# Otherwise, load the regions file specified and draw the region colors on the ground.
		self.region_data = []

		# Load a region file if it has been specified on instantiation.
		if (regionfile!=None):
			loadRegionData(self, regionfile)
			self.region_calib = region_calib

		# Simulation world parameters.
		self.world = ode.World()
		self.world.setGravity((0, -9.81, 0))
		self.world.setERP(0.1)
		self.space = ode.Space()
		self.ground = ode.GeomPlane(space=self.space, normal=(0,1,0), dist=0)

		# CKBot module parameters.
		self.cubesize = 6.0
		self.cubemass = 10.0
		self.hingemaxangle = 1.6
		self.hingeminangle = -1.6
		self.hingemaxforce = 1000000

		# Control parameters.
		self.gait = 0
		self.gain = 1.5
		self.counter = 0

		# Define Modular Arrays
		self.body = []
		self.lowerjoint = []
		self.upperjoint = []
		self.hinge = []
		self.fixed = []
		self.M = []

		# Configuration/Gait Data
		self.config = "none"
		self.connM = []	
		self.basepos = []
		self.baserot = genmatrix(0,1)      	# Identity Matrix for now.
		self.gaits = []
		loadRobotData(self, robotfile)
		
		# Obstacle Data
		self.obstacle = []
		self.obstaclepiece = []
		self.obstacleM = []

		# Account for the "startingpose" argument which translates all the modules
		self.startingpose = startingpose
		if self.startingpose != None:
			tempx = self.basepos[0] + self.startingpose[0]
			tempy = self.basepos[1] + self.startingpose[1]
			tempz = self.basepos[2] + self.startingpose[2]	 
			self.basepos = (tempx, tempy, tempz)

		# Load the objects.
		loadModuleObjects(self)
		self._cjoints = ode.JointGroup()
		
		# Make obstacles if they exist.
		if (obstaclefile!=None):
		    loadObstacles(self, obstaclefile)

		# Create region heights if they are specified.
		self.heightmap = heightmap
		if (self.heightmap!=None):
			loadRegionHeights(self, self.heightmap)


	def _nearcb(self, args, geom1, geom2):
		"""
		Create contact joints between colliding geoms.
		"""

		body1, body2 = geom1.getBody(), geom2.getBody()
		if (body1 is None):
			body1 = ode.environment
		if (body2 is None):
			body2 = ode.environment

		if (ode.areConnected(body1, body2)):
			return

		contacts = ode.collide(geom1, geom2)

		for c in contacts:
			c.setBounce(0.1)
			c.setMu(10000)
			j = ode.ContactJoint(self.world, self._cjoints, c)
			j.attach(body1, body2)

	
	def save_pose_info(self):
		"""
		Adds to the data structure "self.pose_info" with pose information on every module.
		"""
		
		temp = []
		for i in range(len(self.connM)):
			temp.append(get2DPoseAndHeight(self,i))
		self.pose_info.append(temp)	
	
	
	def run(self, MAX_STEPS = None):
		"""
		Run the simulator engine indefinitely or until the self._running flag is set to false.
		"""

		# Initialize parameters.
		self._running = True
		self.pose_info = []
		self.save_pose_info()

		# Receive Locomotion commands for all the hinges from LTLMoP.
		# Use these commands as reference angles for simple P-controlled servos.	
		while self._running and (self.counter < MAX_STEPS or MAX_STEPS == None):
		
			rungait(self)

			# Simulation Step
			self.space.collide((), self._nearcb)
			self.world.step(1/self.fps)
			self._cjoints.empty()
						
			self.counter = self.counter + 1
			self.save_pose_info()
			
			
# Main method.
if (__name__ == '__main__'):
	"""
	Instantiates a simulator and runs it in stand-alone mode with all default arguments.
	"""

	curdir = os.getcwd()
	
	obstaclefile = None
	if len(sys.argv)==3:
            obstaclefile = "../obstacles/" + sys.argv[2] + ".obstacle"

	# Running from ckbot directory.
	if ("simulator" in curdir) and ("ode" in curdir) and ("ckbot" in curdir):
		robotfile = "config/" + sys.argv[1] + ".ckbot"
		sim = CKBotSim(robotfile)
	
	# Running from src.
	else:
		robotfile = "lib/simulator/ode/ckbot/config/" + sys.argv[1] + ".ckbot"
		sim = CKBotSim(robotfile)

	sim.run()
	
