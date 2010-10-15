#!/usr/bin/env python

import ode, xode.parser
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math, time, copy

info = """CKBotSim

CKBot Simulator for LTLMoP
[Sebastian Castro - Cornell University]
[Autonomous Systems Laboratory - 2010]

"""

### TODO:
### 2. Scale the region file with LTLMoP calibration parameters [currently hard-coded].

class CKBotSim:
    """
    CKBot Simulator Class
    """

    fps = 50.0
    cameraDistance = 35.0
    vel = 0.5
    turn = 0.5
    turn2 = 0.5
    clip = 100.0
    res = (640, 480)

    def __init__(self,robotfile, standalone=1,regionfile=None,region_calib=None,startingpose=None):
        """
        Initialize the simulator.
        """

	# Setting standalone to 1 allows for manual key input.
	# Setting standalone to 0 (LTLMoP mode) causes the camera to automatically follow the spawned robot)
	if standalone==1:
	    self.standalone = 1
	else:
	    self.standalone = 0   
	    self.clock = pygame.time.Clock()

	# If regionfile=0, render the ground as default solid green terrain.
        # Otherwise, load the regions file specified and draw the region colors on the ground.
	self.region_data = []

	# Load a region file if it has been specified on instantiation.
	if (regionfile!=None):
	    self.loadRegionData(regionfile)
	    self.region_calib = region_calib

	# Simulation world parameters.
        self._initOpenGL()
	self.world = ode.World()
	self.world.setGravity((0, -9.81, 0))
	self.world.setERP(0.1)
	self.space = ode.Space()
	self.ground = ode.GeomPlane(space=self.space, normal=(0,1,0), dist=0)

	# CKBot module parameters.
	self.cubesize = 6.0
	self.cubemass = 30.0
	self.hingemaxangle = 1.6
	self.hingeminangle = -1.6
	self.hingemaxforce = 5000000

	# Control parameters.
	self.gait = 0
	self.gain = 0.75
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
	self.baserot = self.genmatrix(0,1)      	# Identity Matrix
	self.gaits = []
	self.loadRobotData(robotfile)

	# Account for the "startingpose" argument which translates all the modules
	self.startingpose = startingpose
	if self.startingpose != None:
	    tempx = self.basepos[0] + self.startingpose[0]
	    tempy = self.basepos[1] + self.startingpose[1]
	    tempz = self.basepos[2] + self.startingpose[2]	 
	    self.basepos = (tempx, tempy, tempz)

	# Load the objects.
	self._loadObjects()
        self._cjoints = ode.JointGroup()

        self._xRot = 0.0
        self._yRot = 0.0
        
        self._xCoeff = 360.0 / 480.0
        self._yCoeff = 360.0 / 640.0

        self._vel = 0.0
        self._turn = 0.0
	self._turn2 = 0.0

	# Camera alignment parameters.
	self._ax = 0.0
	self._ay = 0.0
	self._az = 0.0
	self._align = 0.0

    def _loadObjects(self):
        """
	Spawns the CKBot configuration based on the first module's position and orientation.
	All joint angles originally set to zero (0).
	"""  
	
	# Wipe the original lists of bodies, joints, hinges, etc.
	# This step is necessary for reconfiguration, since these lists are non-empty.
	self.body = []
	self.lowerjoint = []
	self.upperjoint = []
	self.hinge = []
	self.fixed = []
	self.M = []	

	# Spawn all modules
	for idx in range(0,len(self.connM)):

	    # Create the lower joint.
	    self.body.append(ode.Body(self.world))
	    self.lowerjoint.append(ode.GeomBox(space=self.space, lengths=(self.cubesize, self.cubesize/2, self.cubesize)))
	    self.lowerjoint[idx].setBody(self.body[2*idx])
	    self.M.append(ode.Mass())
	    self.M[2*idx].setBox(self.cubemass,self.cubesize,self.cubesize/2,self.cubesize)
	    self.body[2*idx].setMass(self.M[2*idx])
 
	    # Create the upper joint.
	    self.body.append(ode.Body(self.world))
	    self.upperjoint.append(ode.GeomBox(space=self.space, lengths=(self.cubesize, self.cubesize/2, self.cubesize)))
	    self.upperjoint[idx].setBody(self.body[2*idx + 1])
	    self.M.append(ode.Mass())
	    self.M[2*idx + 1].setBox(self.cubemass,self.cubesize,self.cubesize/2,self.cubesize)
	    self.body[2*idx + 1].setMass(self.M[2*idx + 1])

	# Set object positions.
	for idx in range(0,len(self.connM)):
	   
            ### If this is the first module spawned (module 0) then set no fixed joints and set the position and orientation according to the self.basepos and self.baserot variables.
	    if (idx==0):

		lowjointadd = self.rotate((0,-self.cubesize/4,0), self.baserot)
		lowjointpos = (self.basepos[0]+lowjointadd[0],
			       self.basepos[1]+lowjointadd[1],
			       self.basepos[2]+lowjointadd[2])
       		self.body[2*idx].setPosition(lowjointpos)
                self.body[2*idx].setRotation(self.baserot)

		upjointadd = self.rotate((0,self.cubesize/4,0), self.baserot)
		upjointpos = (self.basepos[0]+upjointadd[0],
			      self.basepos[1]+upjointadd[1],
			      self.basepos[2]+upjointadd[2])
	        self.body[2*idx + 1].setPosition(upjointpos)
	        self.body[2*idx + 1].setRotation(self.baserot)	

		self.hinge.append(ode.HingeJoint(self.world))
	    	self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
	    	pos1 = self.body[2*idx].getPosition()
	   	pos2 = self.body[2*idx + 1].getPosition()
	   	hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		self.hinge[idx].setAxis(self.rotate((1,0,0),self.baserot))
		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
	   	self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
		self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

	    ### Place all other (subsequent) modules relative to the parent module, and based on the IR ports that define the connection. All these IR Port connections are hard-coded below.
	    else:
		
		# Find which module the current node connects to.
		for j in range(0,idx):	    

		    if (self.connM[idx][j]>0):
			parent = j
			parent_port= self.connM[j][idx]
			child_port = self.connM[idx][j]

			# Create a fixed joint based on the connection type.
			### First, keep track of the parent module's position.
			
			print "Creating connection: \n Modules   %d - %d \n Ports     %d - %d \n" %(parent, idx,parent_port,child_port)
			parent_low_pos = self.body[2*parent].getPosition()
			parent_up_pos = self.body[2*parent + 1].getPosition()
			parent_pos = (0.5*(parent_low_pos[0]+parent_up_pos[0]),
		                      0.5*(parent_low_pos[1]+parent_up_pos[1]),
				      0.5*(parent_low_pos[2]+parent_up_pos[2]))
		
			### Then, rotate and translate the body based on the connection type.
			### These connection strategies are hard-coded in the giant chunk of code below.
			if (parent_port==1):
			    if (child_port==1):

				# CONNECTION 1-1
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((0,-self.cubesize*(0.75),0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi,3))
				self.body[2*idx].setRotation(rot)

				upjointadd = self.rotate((0,-self.cubesize*(1.25),0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((-1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==3):

				# CONNECTION 1-3
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize/4,-self.cubesize,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi/2,3))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((-self.cubesize/4,-self.cubesize,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==5):

				# CONNECTION 1-5
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize/4,-self.cubesize,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi,2),self.genmatrix(-math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((-self.cubesize/4,-self.cubesize,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()
			
				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,-1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==7):

				# CONNECTION 1-7
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((0,-self.cubesize*(1.25),0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				self.body[2*idx].setRotation(rot)

				upjointadd = self.rotate((0,-self.cubesize*(0.75),0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx+1],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==2):
			    if (child_port==2):

				# CONNECTION 2-2
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((0,-self.cubesize*(0.75),0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi),self.genmatrix(math.pi,3))
				self.body[2*idx].setRotation(rot)

				upjointadd = self.rotate((0,-self.cubesize*(1.25),0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==4):

				# CONNECTION 2-4
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize/4,-self.cubesize,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi,2),self.genmatrix(math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((self.cubesize/4,-self.cubesize,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==6):

				# CONNECTION 2-6
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize/4,-self.cubesize,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi,2),self.genmatrix(math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((self.cubesize/4,-self.cubesize,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,-1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==7):

				# CONNECTION 2-7
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((0,-self.cubesize*(1.25),0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(-math.pi/2,2))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((0,-self.cubesize*(0.75),0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx + 1],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,0,1),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==3):
			    if (child_port==1):

				# CONNECTION 3-1
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize*(0.75),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(-math.pi/2,3))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((self.cubesize*(1.25),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,-1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==3):

				# CONNECTION 3-3
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize,self.cubesize/4,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi,3))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((self.cubesize,-self.cubesize,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((-1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==5):

				# CONNECTION 3-5
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize,self.cubesize/4,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi,1))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((self.cubesize,-self.cubesize/4,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==7):

				# CONNECTION 3-7
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize*(1.25),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi/2,3))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((self.cubesize*(0.75),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx + 1],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==4):
			    if (child_port==2):
	
				# CONNECTION 4-2
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize*(0.75),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi,1),self.genmatrix(-math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((self.cubesize*(1.25),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==4):
			
				# CONNECTION 4-4
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize,-self.cubesize/4,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi,2))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((self.cubesize,self.cubesize/4,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((-1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==6):

				# CONNECTION 4-6
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize,-self.cubesize/4,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				self.body[2*idx].setRotation(rot)

				upjointadd = self.rotate((self.cubesize,self.cubesize/4,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==7):

				# CONNECTION 4-7
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize*(1.25),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi/2,1),self.genmatrix(math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((self.cubesize*(0.75),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx + 1],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,0,1),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==5):
			    if (child_port==1):

				# CONNECTION 5-1
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize*(0.75),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi,1),self.genmatrix(math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((-self.cubesize*(1.25),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,-1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==3):
			
				# CONNECTION 5-3
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize,self.cubesize/4,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi,1))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((-self.cubesize,-self.cubesize/4,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==5):

				# CONNECTION 5-5
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize,self.cubesize/4,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi,3))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((-self.cubesize,-self.cubesize/4,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()
		
				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((-1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==7):

				# CONNECTION 5-7
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize*(1.25),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(-math.pi/2,3),self.genmatrix(math.pi,2))
				rot3 = self.multmatrix(rot,rot2)
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((-self.cubesize*(0.75),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx + 1],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==6):
			    if (child_port==2):

				# CONNECTION 6-2
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize*(0.75),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi/2,3))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((-self.cubesize*(1.25),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==4):
			
				# CONNECTION 6-4
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize,-self.cubesize/4,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				self.body[2*idx].setRotation(rot)

				upjointadd = self.rotate((-self.cubesize,self.cubesize/4,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)
		
			    elif (child_port==6):
			
				# CONNECTION 6-6
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize,-self.cubesize/4,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi,2))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((-self.cubesize,self.cubesize/4,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((-1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==7):

				# CONNECTION 6-7
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize*(1.25),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi/2,1),self.genmatrix(-math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((-self.cubesize*(0.75),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx + 1],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,0,-1),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==7):
			    if (child_port==1):

				# CONNECTION 7-1
				rot = self.body[2*parent + 1].getRotation()
				lowjointadd = self.rotate((0,self.cubesize*(0.75),0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				self.body[2*idx].setRotation(rot)

				upjointadd = self.rotate((0,self.cubesize*(1.25),0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent + 1])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==2):

				# CONNECTION 7-2
				rot = self.body[2*parent + 1].getRotation()
				lowjointadd = self.rotate((0,self.cubesize*(0.75),0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi/2,2))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((0,self.cubesize*(1.25),0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent+1])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,0,-1),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==3):

				# CONNECTION 7-3
				rot = self.body[2*parent + 1].getRotation()
				lowjointadd = self.rotate((-self.cubesize/4,self.cubesize,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(-math.pi/2,3))
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((self.cubesize/4,self.cubesize,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,-1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==4):

				# CONNECTION 7-4
				rot = self.body[2*parent + 1].getRotation()
				lowjointadd = self.rotate((0,self.cubesize,self.cubesize/4),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi/2,2),self.genmatrix(-math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)			
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((0,self.cubesize,-self.cubesize/4),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx+1],self.body[2*parent+1])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,-1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==5):

				# CONNECTION 7-5
				rot = self.body[2*parent + 1].getRotation()
				lowjointadd = self.rotate((-self.cubesize/4,self.cubesize,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi,2),self.genmatrix(math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)			
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((self.cubesize/4,self.cubesize,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==6):

				# CONNECTION 7-6
				rot = self.body[2*parent + 1].getRotation()
				lowjointadd = self.rotate((0,self.cubesize,self.cubesize/4),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi/2,2),self.genmatrix(math.pi/2,3))
				rot3 = self.multmatrix(rot,rot2)			
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((0,self.cubesize,-self.cubesize/4),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==7):

				# CONNECTION 7-7
				rot = self.body[2*parent + 1].getRotation()
				lowjointadd = self.rotate((0,self.cubesize*(1.25),0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi,3))		
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((0,self.cubesize*(0.75),0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx + 1],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((-1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==8):
			    if (child_port==8):

				# CONNECTION 8-8 [aka 1-2 & 2-1]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((0,-self.cubesize*(0.75),0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(-math.pi/2,2),self.genmatrix(math.pi,3))	
				rot3 = self.multmatrix(rot,rot2)	
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((0,-self.cubesize*(1.25),0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,0,-1),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    if (child_port==9):

				# CONNECTION 8-9 [aka 1-4 & 2-3]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((0,-self.cubesize,self.cubesize/4),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(-math.pi/2,2),self.genmatrix(math.pi/2,3))	
				rot3 = self.multmatrix(rot,rot2)	
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((0,-self.cubesize,-self.cubesize/4),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    if (child_port==10):

				# CONNECTION 8-10 [aka 1-6 & 2-5]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((0,-self.cubesize,self.cubesize/4),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi/2,2),self.genmatrix(-math.pi/2,3))	
				rot3 = self.multmatrix(rot,rot2)	
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((0,-self.cubesize,-self.cubesize/4),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,-1,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==9):
			    if (child_port==8):

				# CONNECTION 9-8 [aka 3-2 & 4-1]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize*(0.75),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi/2,1),self.genmatrix(-math.pi/2,3))	
				rot3 = self.multmatrix(rot,rot2)	
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((self.cubesize*(1.25),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,0,-1),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==9):
			
				# CONNECTION 9-9 [aka 3-4 & 4-3]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize,0,self.cubesize/4),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(-math.pi/2,1),self.genmatrix(math.pi,2))	
				rot3 = self.multmatrix(rot,rot2)	
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((self.cubesize,0,-self.cubesize/4),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()	

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((-1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==10):

				# CONNECTION 9-10 [aka 3-6 & 4-5]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((self.cubesize,0,self.cubesize/4),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(-math.pi/2,1))		
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((self.cubesize,0,-self.cubesize/4),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			elif (parent_port==10):
			    if (child_port==8):

				# CONNECTION 10-8 [aka 6-1 & 5-2]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize*(0.75),0,0),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi/2,1),self.genmatrix(math.pi/2,3))	
				rot3 = self.multmatrix(rot,rot2)	
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((-self.cubesize*(1.25),0,0),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((0,0,1),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==9):

				# CONNECTION 10-9 [aka 6-3 & 5-4]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize,0,-self.cubesize/4),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(rot,self.genmatrix(math.pi/2,1))		
				self.body[2*idx].setRotation(rot2)

				upjointadd = self.rotate((-self.cubesize,0,self.cubesize/4),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot2)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

			    elif (child_port==10):

				# CONNECTION 10-10 [aka 6-5 & 5-6]
				rot = self.body[2*parent].getRotation()
				lowjointadd = self.rotate((-self.cubesize,0,-self.cubesize/4),rot)
				lowjointpos = (parent_pos[0]+lowjointadd[0],
					       parent_pos[1]+lowjointadd[1],
					       parent_pos[2]+lowjointadd[2])
			    	self.body[2*idx].setPosition(lowjointpos)
				rot2 = self.multmatrix(self.genmatrix(math.pi/2,1),self.genmatrix(math.pi,2))	
				rot3 = self.multmatrix(rot,rot2)	
				self.body[2*idx].setRotation(rot3)

				upjointadd = self.rotate((-self.cubesize,0,self.cubesize/4),rot)
				upjointpos = (parent_pos[0]+upjointadd[0],
					      parent_pos[1]+upjointadd[1],
					      parent_pos[2]+upjointadd[2])
				self.body[2*idx+1].setPosition(upjointpos)
				self.body[2*idx+1].setRotation(rot3)

				self.fixed.append(ode.FixedJoint(self.world))
				self.fixed[idx-1].attach(self.body[2*idx],self.body[2*parent])
				self.fixed[idx-1].setFixed()

				self.hinge.append(ode.HingeJoint(self.world))
		    		self.hinge[idx].attach(self.body[2*idx],self.body[2*idx + 1])
		    		pos1 = self.body[2*idx].getPosition()
		   		pos2 = self.body[2*idx + 1].getPosition()
		   		hingepos = (0.5*(pos1[0]+pos2[0]),0.5*(pos1[1]+pos2[1]),0.5*(pos1[2]+pos2[2]))
		    		self.hinge[idx].setAnchor(hingepos)
		    		self.hinge[idx].setAxis(self.rotate((-1,0,0),rot))
		    		self.hinge[idx].setParam(ode.ParamLoStop,self.hingeminangle)
		   		self.hinge[idx].setParam(ode.ParamHiStop,self.hingemaxangle)
				self.hinge[idx].setParam(ode.ParamFMax,self.hingemaxforce)

	# Build the Geoms and Joints arrays for rendering.
	self._geoms = self.lowerjoint
	self._geoms.extend(self.upperjoint)
	self._geoms.append(self.ground)
	self._joints = self.fixed
	self._joints.extend(self.hinge)

    def loadRegionData(self, regionfile):
	"""
	Parses a LTLMoP-formatted ".regions" file and creates a data structure to render the region map in the simulator.
	"""

        data = open(regionfile,"r")

        reading_regions = 0
        for line in data:

            # Find when the Regions information is found in the text file (as per LTLMoP format)
            if (line.split().count("Regions:")>0):
	        reading_regions = 1
            # Once we have found where the region information begins, parse it in the form:
            # [color info, vertex info] ==> [R, G, B, [x1, y1], [x2, y2], ...] 
            elif (reading_regions == 1 ):
	        info = line.split()
                if (len(info)==0):
	            reading_regions = 0
 	        else:
	            region_color = [float(info[6])/255, float(info[7])/255, float(info[8])/255]
		    posx = int(info[2])
		    posy = int(info[3])
	            vertices = []
	            for idx in range(9,len(info),2):
	                vertices.append([posx + int(info[idx]), posy + int(info[idx+1])])
	            temp_info = region_color
	            temp_info.extend(vertices)
	            self.region_data.append(temp_info)

    def loadRobotData(self,filename):
	"""
	Loads full robot information from a text file that specifies:
	 1. Configuration Matrix.
	 2. Relative rotation and translation from the origin, in CKBot length units.
	 3. A set of gaits and target gait execution times.
	"""

	# Clear all previous information
	self.config = "none"
	self.connM = []
	self.basepos = []
	self.baserot = self.genmatrix(0,1)   # Identity Matrix
	self.gaits = []

	# Open the text file.
	data = open(filename,"r")

	# Go through the text file line by line.
	reading = "none"
	for line in data:
	    linesplit = line.split()

	    # If we are currently reading nothing, then we are looking for the next attribute to read.
	    if reading == "none" and linesplit != []:
	        if linesplit[0] == "ConfigName:":
		    reading = "name"
	        elif linesplit[0] == "ConnMatrix:":
		    reading = "matrix"
		elif linesplit[0] == "RelativeOffset:":
		    reading = "offset"
	        elif linesplit[0] == "RelativeRotation:":
		    reading = "rotation"
                elif linesplit[0] == "Gaits:":
		    reading = "gait"

	    # If we are currently reading something, the continue to do so until finished.

	    # If we are reading the name of the configuration, then it is simply the word that makes the line under "ConfigName:"
	    elif reading == "name":
		self.config = linesplit[0]
		reading = "none"

	    # If we are reading the connectivity matrix, then read the matrix row by row until we reach whitespace.
	    elif reading == "matrix":
		if linesplit == []:
		    reading = "none"
		else:
		    temprow = []
		    for num in linesplit:
			temprow.append(int(num))
		    self.connM.append(temprow)

	    # If we are reading the relative offset, then simply read the three numbers (x, y, z) in 
            # the line under "RelativeOffset:" and scale by the size of the modules in the simulator.
	    elif reading == "offset":
		for num in linesplit:
		    self.basepos.append(self.cubesize*float(num))
		reading = "none"

	    # If we are reading the relative rotation, then we must read through all the rotations specified and
	    # create rotation matrices for each rotation and multiply them together in the correct order.
	    elif reading == "rotation":
		if linesplit == []:
		    reading = "none"
		else:
		    angle = int(linesplit[0])*(math.pi/180)
		    axis = linesplit[1]
		    if axis == "x":
			axis_idx = 1
		    elif axis == "y":
			axis_idx = 2
		    else:
			axis_idx = 3
		    self.baserot = self.multmatrix(self.genmatrix(angle,axis_idx),self.baserot)

	    # If we are reading gaits, it is more complicated. We must ensure to read all the gaits specified.
	    # For each gait, we must be able to tell whether the gait is Periodic or Fixed type and 
	    # parse it accordingly.

	    elif reading == "gait":

		# Read the Proportional control gain specified in the text file.
		if linesplit[0] == "Gain":
		    self.gain = float(linesplit[1])

		# Figure out whether the gaits are fixed or periodic
		if linesplit[0] == "Type":
		    self.gaittype = linesplit[1]
		    if self.gaittype == "Periodic":
			reading = "periodic_gait"
		    else:
			reading = "fixed_gait"				  
				
	    # If we are reading periodic gaits, we know our gait table is just 3 lines.
	    # The first line is the set of amplitudes (in degrees*100) of each hinge.
	    # The second line is the set of frequencies (in rad/s) of each hinge.
	    # The third line is the set of phase angles (in degrees*100) of each hinge.
	    elif reading == "periodic_gait" and linesplit != []:
		if linesplit[0] == "Gait":
		    amplitudes = []
		    frequencies = []
		    phases = []
		    reading = "amplitude"

	    elif reading == "amplitude":
		for elem in linesplit:
		    amplitudes.append( float(elem)*(math.pi/180.0)*(1/100.0) )
		reading = "frequency"

	    elif reading == "frequency":
		for elem in linesplit:
		    frequencies.append( float(elem) )
		reading = "phase"

	    elif reading == "phase":
		for elem in linesplit:
		    phases.append( float(elem)*(math.pi/180.0)*(1/100.0) )
		tempgait = [amplitudes]
		tempgait.append(frequencies)
		tempgait.append(phases)
		self.gaits.append(tempgait)
		reading = "periodic_gait"

	    # If we are reading fixed gaits, the gait table can be an arbitrary number of lines.
	    # For each gait we will read all the steps until we find the last line for the gait 
            # (which the time that the gait should loop in).
	    elif reading == "fixed_gait" and linesplit != []:
		if linesplit[0] == "Gait":
		    gaitrows = []
		    gaittime = 0
		    reading = "fixed_gait_rows"

	    elif reading == "fixed_gait_rows":
		if len(linesplit)==1:
		    gaittime = [float(linesplit[0])]
		    gaittime.extend(gaitrows)
		    self.gaits.append(gaittime)
		    reading = "fixed_gait"
		else:
		    temprow = []
		    for elem in linesplit:
		        temprow.append( float(elem)*(math.pi/180.0)*(1/100.0) )
		    gaitrows.append(temprow)		    

    def rotate(self,vec,rot):
	"""
	Rotates vector 'vec' using matrix 'mat'
	"""
	
	item1 = rot[0]*vec[0] + rot[1]*vec[1] + rot[2]*vec[2]
	item2 = rot[3]*vec[0] + rot[4]*vec[1] + rot[5]*vec[2]
	item3 = rot[6]*vec[0] + rot[7]*vec[1] + rot[8]*vec[2]
	newvec = [item1, item2, item3]

	for i in range(0,3):
	    if newvec[i]<1e-5 and newvec[i]>-1e-5:
		newvec[i] = 0

	return tuple(newvec)

    def genmatrix(self,angle,axis):
	"""
	Generates a rotation matrix about the specified axis
	"""
	
	if (axis==1):	# X-AXIS
	    rot = (1, 0, 0, 0, math.cos(angle), math.sin(angle), 0, -math.sin(angle), math.cos(angle))
	elif (axis==2):	# Y-AXIS
	    rot = (math.cos(angle), 0, -math.sin(angle), 0, 1, 0, math.sin(angle), 0, math.cos(angle))
	elif (axis==3): # Z-AXIS
	    rot = (math.cos(angle), math.sin(angle), 0, -math.sin(angle), math.cos(angle), 0, 0 , 0, 1)

	return rot

    def multmatrix(self,M1,M2):
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

    def _initOpenGL(self):
        """
        Initialise the scene.
        """
        
        # Create a window
        pygame.init()
        screen = pygame.display.set_mode(self.res,
                                         pygame.OPENGL | pygame.DOUBLEBUF)
        pygame.display.set_caption('CKBot Simulation')
        pygame.mouse.set_visible(False)

        glViewport(0, 0, self.res[0], self.res[1])
        glClearColor(0.8, 0.8, 0.9, 0)
        
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_FLAT)

    def _extractMatrix(self, geom):
        """
        Return a 4x4 matrix (represented by a 16-element tuple) created by
        combining the geom's rotation matrix and position.
        """
        
        x, y, z = geom.getPosition()
        rot = geom.getRotation()
        return (rot[0], rot[3], rot[6], 0.0,
                rot[1], rot[4], rot[7], 0.0,
                rot[2], rot[5], rot[8], 0.0,
                x, y, z, 1.0)

    def _renderGeom(self, geom, color):
        """
        Render either a ode.GeomBox or ode.GeomSphere object.
        """

        allowed = [ode.GeomBox, ode.GeomSphere]
        ok = False
        for klass in allowed:
            ok = ok or isinstance(geom, klass)
        if (not ok):
            return

        glPushMatrix()
        glMultMatrixd(self._extractMatrix(geom))

        glMaterialfv(GL_FRONT, GL_SPECULAR, color)

        if (isinstance(geom, ode.GeomBox)):
            sx, sy, sz = geom.getLengths()
            glScale(sx, sy, sz)
            glutInit()
            glutSolidCube(1)
        elif (isinstance(geom, ode.GeomSphere)):
            r = geom.getRadius()
            glutSolidSphere(r, 20, 20)

        glPopMatrix()

    def _renderGround(self):
        """
        Renders the ground plane.
        """

        normal, d = self.ground.getParams()
        x, y, z = self.lowerjoint[0].getPosition()

	
        # Draw a quad at the position of the vehicle that extends to the
        # clipping planes.
	if (self.region_data==[]):
	
            glPushMatrix()
            glTranslate(x, 0.0, z)

            glMaterialfv(GL_FRONT, GL_SPECULAR, (0.0, 1.0, 0.0))

            glBegin(GL_QUADS)
            glNormal3f(*normal)
            glVertex3f(-self.clip, d, -self.clip)
            glNormal3f(*normal)
            glVertex3f(self.clip, d, -self.clip)
            glNormal3f(*normal)
            glVertex3f(self.clip, d, self.clip)
            glNormal3f(*normal)
            glVertex3f(-self.clip, d, self.clip)
            glEnd()

            glPopMatrix()

	# If we have region data, draw the individual regions and color them accordingly.
	else:
	    for row in self.region_data:
	        glPushMatrix()

                color = (row[0], row[1], row[2])
                glMaterialfv(GL_FRONT, GL_SPECULAR, color)

                glBegin(GL_POLYGON)
		for idx in range(3,len(row)):
        	    glNormal3f(*normal)
        	    glVertex3f(row[idx][0]*self.region_calib[0], d, -row[idx][1]*self.region_calib[1])
        	glEnd()

                glPopMatrix()

    def _setCamera(self):
        """
        Position the camera to C{self.cameraDistance} units behind the
        vehicle's current position and rotated depending on the mouse position.
        """

        aspect = float(self.res[0]) / float(self.res[1])
        
        x, y = pygame.mouse.get_pos()
        self._xRot = (y - self.res[1]/2) * self._xCoeff
        self._yRot = (x - self.res[0]/2) * self._yCoeff
        if (self._xRot < 0):
            self._xRot = 0
                
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-aspect, aspect, -1.0, 1.0, 1.5, self.clip)

        glLightfv(GL_LIGHT0, GL_POSITION, (-5.0, 10.0, 0, 0))
        glLightfv(GL_LIGHT0,GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0,GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))
        glEnable(GL_LIGHT0)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Set the camera angle to view the vehicle
        glTranslate(0.0, 0.0, -self.cameraDistance)
        glRotate(self._xRot, 1, 0, 0)
        glRotate(self._yRot, 0, 1, 0)

        # Set the camera so that the vehicle is drawn in the correct place.
	if (self._align==1 or self.standalone==0):
            x, y, z = self.lowerjoint[0].getPosition()
	    self._ax = x
	    self._ay = y
	    self._az = z
        glTranslate(-self._ax, -self._ay, -self._az)

    def render(self):
        """
        Render the current simulation state.
        """
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._renderGround()

        self._setCamera()
	counter = 1;
        for geom in self._geoms:
            self._renderGeom(geom,(0,0,0))

        glFlush()
        pygame.display.flip()

    def _keyDown(self, key):
        if (key == pygame.K_w):
            self._vel = self.vel
        elif (key == pygame.K_a):
            self._turn = -self.turn
        elif (key == pygame.K_d):
            self._turn = self.turn
        elif (key == pygame.K_s):
            self._vel = -self.vel
	elif (key == pygame.K_q):
	    self._turn2 = self.turn2
	elif (key == pygame.K_e):
	    self._turn2 = -self.turn2
        elif (key == pygame.K_ESCAPE):
            self._running = False
	elif (key == pygame.K_l):
	    self._align = 1.0

	# If running in standalone mode, can press the keys 1-5 to set gaits 1-5.
	# Yes, only 5 gaits are supported but if you need to add more simply extend the code below!
	elif (key == pygame.K_0):
	    self.gait = 0
	elif (key == pygame.K_1):
	    if len(self.gaits) >= 1:
		self.gait = 1
	elif (key == pygame.K_2):
	    if len(self.gaits) >= 2:
		self.gait = 2
	elif (key == pygame.K_3):
	    if len(self.gaits) >= 3:
		self.gait = 3
	elif (key == pygame.K_4):
	    if len(self.gaits) >= 4:
		self.gait = 4
	elif (key == pygame.K_5):
	    if len(self.gaits) >= 5:
		self.gait = 5

    def _keyUp(self, key):
        if (key == pygame.K_w):
            self._vel = 0.0
        elif (key == pygame.K_a):
            self._turn = 0.0
        elif (key == pygame.K_d):
            self._turn = 0.0
        elif (key == pygame.K_s):
            self._vel = 0.0
	elif (key == pygame.K_q):
	    self._turn2 = 0.0
	elif (key == pygame.K_e):
	    self._turn2 = 0.0
	elif (key == pygame.K_l):
	    self._align = 0.0
	
    def doEvents(self):
        """
        Process any input events.
        """
        
        events = pygame.event.get()
        
        for e in events:
            if (e.type == pygame.QUIT):
                self._running = False
            elif (e.type == pygame.KEYDOWN):
                self._keyDown(e.key)
            elif (e.type == pygame.KEYUP):
                self._keyUp(e.key)
            elif (e.type == pygame.MOUSEBUTTONDOWN):
                if (e.button == 1):
                    self.lowerjoint.addForce((0.0, 500000, 0.0))

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
            c.setBounce(0.2)
            c.setMu(10000)
            j = ode.ContactJoint(self.world, self._cjoints, c)
            j.attach(body1, body2)

    def get2DPose(self, num):
	"""
	Get the 2D Pose (x, y, yaw) of module 'num'
	"""

	# Find 2-D pose using module num's position and rotation matrix.
	# NOTE: Due to the definition of the axes, the (x,y) position is actually (x,-z)
	pose2d = [0,0,0]
	# X and Y values are the average of the lower and upper parts of the base module.
	pose2d[0] = 0.5*(self.lowerjoint[num].getPosition()[0] + self.upperjoint[num].getPosition()[0])
	pose2d[1] = -0.5*(self.lowerjoint[num].getPosition()[2] + self.upperjoint[num].getPosition()[2])
	# Find 2D Orientation using the rotation matrix and the x-direction unit vector.
	# Rotate the vector [1, 0, 0] using the rotation matrix and find the angle to the x-axis using the atan2 function (to cover all 4 quadrants)
	unitvec = [1,0,0]
	rot = self.lowerjoint[num].getRotation()
	rotvec = self.rotate(unitvec,rot)
	pose2d[2] = math.atan2(-rotvec[2],rotvec[0]) 

	return pose2d

    def reconfigure(self, name):
	"""
	Removes the previous CKBot configuration and spawns a new one as specified in the function arguments.
	"""

	# Load the new robot data from the new file "name".ckbot.
	print("==========\nReconfiguring: " + name + "\n==========")
	robotfile = "CKBot/" + name + ".ckbot"
	self.loadRobotData(robotfile)

	# Update the position of the base module (always module 0) as the sum of the current configuration's pose
	# and the default base position matrix for a given configuration.
	relpos = [0.5*(self.lowerjoint[0].getPosition()[0] + self.upperjoint[0].getPosition()[0]),
		  0, 0.5*(self.lowerjoint[0].getPosition()[2] + self.upperjoint[0].getPosition()[2])]
	self.basepos = [relpos[0]+self.basepos[0], relpos[1]+self.basepos[1], relpos[2]+self.basepos[2]]	

	# Reload the objects.
	self._loadObjects()

    def setGait(self,gait):
	"""
	Set the gait number for simulation
	"""
	self.gait = gait

    def rungait(self):
	"""
	Runs the gait specified by the object variable "self.gait"
	"""
	
	time = self.counter/self.fps
	gait = self.gaits[self.gait - 1]

	# If the gait is set to zero, stop moving all hinges.
	if self.gait == 0:
	   for module_idx in range(len(self.hinge)):
		self.hinge[module_idx].setParam(ode.ParamVel, 0)

	else:
	    # If the gait is of periodic type, read the rows in the following format.
	    # ROW 1: Amplitudes
	    # ROW 2: Frequencies
	    # ROW 3: Phases
	    if self.gaittype == "Periodic":
	        for module_idx in range(len(self.hinge)):
		    amplitude = gait[0][module_idx]
		    frequency = gait[1][module_idx]
		    phase = gait[2][module_idx]

	            true_ang = self.hinge[module_idx].getAngle()
		    ref_ang = amplitude*math.sin(frequency*time + phase)
		    servo_vel = self.gain*(ref_ang-true_ang)
	            self.hinge[module_idx].setParam(ode.ParamVel, servo_vel)

	    # If the gait is of fixed type, run the function "gaitangle" to interpolate between
	    # reference hinge angles at the current time.
	    else:
	        for module_idx in range(len(self.hinge)):
		    true_ang = self.hinge[module_idx].getAngle()	    
 		    ref_ang = self.gaitangle(gait,time,module_idx)
		    servo_vel = self.gain*(ref_ang-true_ang)
	            self.hinge[module_idx].setParam(ode.ParamVel, servo_vel)

    def gaitangle(self,gait,time,module):
	"""
	Takes in a gait matrix and returns the reference angle at that point in time.
	"""

	nummoves = len(gait)-1
	gaittime = gait[0]
	singletime = float(gaittime)/(nummoves-1)

	timearray = []
	for i in range(0,nummoves):
	    if i==0:
		timearray.append(0)
	    else:
		timearray.append(timearray[i-1]+singletime)
	currenttime = (time%gaittime)/singletime

        globalref = gait[int(math.ceil(currenttime))+1][module]
	globalprev = gait[int(math.floor(currenttime))+1][module]

	if globalref==globalprev:
	    localref = globalref
        else:
	    interp = (currenttime - timearray[int(math.floor(currenttime))])/(timearray[int(math.ceil(currenttime))] - timearray[int(math.floor(currenttime))])
	    localref = globalprev + interp*(globalref-globalprev)

	return globalref

    def run(self):
        """
        Start the demo. This method will block until the demo exits.
        This method is used if the simulator is run stand-alone.
        """

        self.clock = pygame.time.Clock()
        self._running = True
        self.doEvents()

        # Receive Locomotion commands for all the hinges from LTLMoP.
        # Use these commands as reference angles for simple P-controlled servos.	
        while self._running:

		self.doEvents()

	    	# Servo Behavior with Gaits [ YAY CONTROLS ! ]
		counter = self.counter
		
		self.rungait()
	
	        # Simulation Step
	        self.space.collide((), self._nearcb)
	        self.world.step(1/self.fps)
	        self._cjoints.empty()
	        self.render()

   	        # Limit the FPS.
	        self.clock.tick(self.fps)
	        self.counter = self.counter + 1

    def run_once(self):
        """
        Run one simulation step -- used for LTLMoP integration.
        """

        self._running = True
        self.doEvents()

        # Receive Locomotion commands for all the hinges from LTLMoP.
        # Use these commands as reference angles for simple P-controlled servos.
	self.rungait()

    	# Simulation Step
    	self.space.collide((), self._nearcb)
    	self.world.step(1/self.fps)
    	self._cjoints.empty()
    	self.render()

    	# Limit the FPS.
    	self.clock.tick(self.fps)
    	self.counter = self.counter + 1

if (__name__ == '__main__'):
    """
    Instantiates a simulator and runs it in stand-alone mode with all default arguments.
    """

    print info

    ### FOR NOW, SET THE FILENAME HERE
    robotfile = "CKBot/Slinky.ckbot"

    sim = CKBotSim(robotfile, standalone=0)
    sim.run()
