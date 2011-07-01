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

info = """Gait Creator

CKBot Gait Creator for LTLMoP
[Sebastian Castro - Cornell University]
[Autonomous Systems Laboratory - 2011]

Q/W : Cycle through modules (highlighted in red)
A/S : Change selected joint angle (Large Step)
Z/X : Change Selected joint angle (Small Step)
 O  : Reset joint angles to zero

 R  : "Record" new gait
 C  : "Capture" current frame
 D  : "Done" -- Finish recording gait and save
 N  : "Null" -- Cancel current recording

"""

class GaitCreator:
	"""
	CKBot Gait Creator Class
	"""

	fps = 30.0
	cameraDistance = 40.0
	vel = 0.5
	turn = 0.5
	turn2 = 0.5
	clip = 1000.0
	res = (800, 600)

	def __init__(self,robotfile, standalone=1,obstaclefile=None,regionfile=None,region_calib=None,startingpose=None,heightmap=None):
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
		#regionfile = "examples/sandbox/sandbox.regions"
		#region_calib = [0.1,-0.1]
		if (regionfile!=None):
			loadRegionData(self, regionfile)
			self.region_calib = region_calib

		# Simulation world parameters.
		self._initOpenGL()
		self.world = ode.World()
		self.world.setGravity((0, -9.81, 0))
		self.world.setERP(0.25)
		self.space = ode.Space()
		self.ground = ode.GeomPlane(space=self.space, normal=(0,1,0), dist=0)

		# CKBot module parameters.
		self.cubesize = 6.0
		self.cubemass = 10.0
		self.hingemaxangle = 1.6
		self.hingeminangle = -1.6
		self.hingemaxforce = 2500000

		# Control parameters.
		self.gait = 0
		self.gain = 0
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
		self.robotfile = robotfile
		loadRobotData(self, self.robotfile)
		
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
		self.clicking = False
		
		# Gait Creator parameters.
		self.current_module = 0
		self.num_modules = len(self.connM)
		self.ref_angles = []
		for i in range(self.num_modules):
			self.ref_angles.append(0)
		self.recording = False
		self.saved_frame = []
		
		# Make obstacles if they exist.
		if (obstaclefile!=None):
		    loadObstacles(self, obstaclefile)

		# Create region heights if they are specified.
		self.heightmap = heightmap
		if (self.heightmap!=None):
			loadRegionHeights(self, self.heightmap)
	

	def _initOpenGL(self):
		"""
		Initialise the scene.
		"""

		# Create a window
		pygame.init()
		screen = pygame.display.set_mode(self.res, pygame.OPENGL | pygame.DOUBLEBUF)
		pygame.display.set_caption('CKBot Gait Creator')
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

			glMaterialfv(GL_FRONT, GL_SPECULAR, (0.0, 0.5, 0.0))

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
			
			# Render squares on the ground in order to observe motion of the robot.
			for i in range(-10,11,2):
				for j in range(-10,11,2):
					glPushMatrix()
					color = (0, 0, 0.5)
					glMaterialfv(GL_FRONT, GL_SPECULAR, color)
					glBegin(GL_POLYGON)
					glNormal3f(*normal)
					glVertex3f(-10 + 20*i, 0.1, -10 + 20*j)
					glNormal3f(*normal)
					glVertex3f(-10 + 20*i, 0.1, 10 + 20*j)
					glNormal3f(*normal)
					glVertex3f(10 + 20*i, 0.1, 10 + 20*j)
					glNormal3f(*normal)
					glVertex3f(10 + 20*i, 0.1, -10 + 20*j)
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
		if self.clicking:
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
		counter = 0;
		counter = 0;
		for geom in self._geoms:
			if (counter == self.current_module) or (counter == self.current_module + self.num_modules):
				self._renderGeom(geom,(1,0,0))
			else:
				self._renderGeom(geom,(0,0,0))
			counter = counter + 1 
		glFlush()
		pygame.display.flip()


	def _keyDown(self, key):
		# Gait commands.
		# USE Q-W to cycle through modules (backwards and forward respectively)
		if (key == pygame.K_q and self.current_module < self.num_modules - 1):
			self.current_module = self.current_module + 1;
		elif (key == pygame.K_w and self.current_module > 0):
			self.current_module = self.current_module - 1;

		# USE A-S to move the currently selected joint angle [Large Step]
		elif (key == pygame.K_a):
			self.ref_angles[self.current_module] = min([self.ref_angles[self.current_module] + 1000, 9000])
		elif (key == pygame.K_s):
			self.ref_angles[self.current_module] = max([self.ref_angles[self.current_module] - 1000, -9000])

		# USE Z-X to move the currently selected joint angle [Small Step]
		elif (key == pygame.K_z):
			self.ref_angles[self.current_module] = min([self.ref_angles[self.current_module] + 250, 9000])
		elif (key == pygame.K_x):
			self.ref_angles[self.current_module] = max([self.ref_angles[self.current_module] - 250, -9000])		
			
		# Reset to zero position by pressing O
		elif (key == pygame.K_o):
			print "Resetting Joint Angles"
			for i in range(self.num_modules):
				self.ref_angles[i] = 0

		# Gait Recording:
		# Press R to enable recording a gait.
		elif (key == pygame.K_r and not self.recording):
			self.recording = True
			self.temp_recording = []
			self.gait_frame = 1
			print "Recording Gait"
		# Press C to capture a frame while recording.
		elif (key == pygame.K_c and self.recording):
			self.temp_recording.append(copy.deepcopy(self.ref_angles))
			print "Gait Frame %i Recorded" % self.gait_frame
			self.gait_frame = self.gait_frame + 1
		# Press D to save a gait as the next gait in the text file.
		elif (key == pygame.K_d and self.recording):
			# Determine gait time
			gaittime = input('In how many seconds should this gait execute? Enter number: ')
			self.temp_recording.append(gaittime)
			# Determine gait name
			self.gaitname = raw_input('Name this gait: ')
			# Get gait traits
			self.traits = raw_input("Enter gaits separate by a comma and a space (e.g. fast, tall): ")
			# Update Library
			# self.updateLibe()
			# Save Gait File
			self.saveGait()
			print "Gait Saved"
			self.recording = False	
		# Press N to cancel recording.
		elif (key == pygame.K_n and self.recording):
			self.recording = False
			print "Recording Cancelled\n"
		
		elif (key == pygame.K_ESCAPE):
			self._running = False
		elif (key == pygame.K_l):
			self._align = 1.0

		# If running in standalone mode, can press the keys 1-5 to set gaits 1-5.
		# Yes, only 5 gaits are supported but if you need to add more simply extend the code below!
		if not self.recording:
			if (key == pygame.K_0):
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
				if e.button == 1:
					self.clicking = True
				elif e.button == 4:
					self.cameraDistance -= 5
				elif e.button == 5:
					self.cameraDistance += 5
			elif (e.type == pygame.MOUSEBUTTONUP):
				if e.button == 1:
					self.clicking = False


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
	
	def saveGait(self):
		"""
		Saves a recently recorded gait by adding it to the currently loaded .ckbot file
		"""
		
		# Load the robot file.
		f = open(self.robotfile, 'a')

		# Add new gait name
		gaitname = self.gaitname
		namestring = '\n\n# Gait name: ' + gaitname
		f.write(namestring)

		# Add traits
		traitsstring = '\n# Traits: ' + self.traits
		f.write(traitsstring)
		
		# Add the new gait number
		numgaits = len(self.gaits)
		titlestring = "\nGait " + str(numgaits + 1) + ":\nType Fixed"
		f.write(titlestring)

		# Now add each row of the gait file
		for line in self.temp_recording:
			linestring = "\n"
			if type(line) == int:	# This deals with single-element lines.
				linestring = linestring + str(line)
			else:					# This corresponds to any other line.
				for elem in line:
					linestring = linestring + str(elem) + " "
			f.write(linestring)
		
		# Close the file.
		f.close()
		
		# Reload the robot data.
		loadRobotData(self, self.robotfile)		

	def updateLibe(self):
		"""
		Update CKBotTraits.libe with the new gait and traits
		"""
		gaitname = self.gaitname
		traits = self.traits.split(", ")
		traitPreexists = []
		while len(traitPreexists) < len(traits):
			traitPreexists.append(0)
		config = sys.argv[1]

		# Rename file to have a ~ at the end
		libFile = 'library/CKBotTraits.libe'
		os.rename(libFile,libFile+'~')

		# Load the robot file.
		destination= open(libFile, 'w' )
		source= open(libFile+'~', 'r' )

		# Append config-gait pair to trait list if it already exists
		for line in source:
			if not ('# LEAVE THIS LINE HERE (one enter line below last trait list) for correct parsing' in line):
				destination.write(line)
				if 'Trait: ' in line:
					idx = 0
					for trait in traits:
						if trait in line:
							destination.write( config + '-' + gaitname + '\n' )
							traitPreexists[idx] = 1
						idx = idx + 1


		# Make a new trait list in the library if it doesn't already exist
		idx = 0
		for trait in traits:
			if traitPreexists[idx] == 0:
				print "New trait! Please write a short definition for trait " + trait + ":"
				newdefn = raw_input()
				# Make a new trait list
				destination.write('# \"' + trait + '\" = ' + newdefn + '\n')
				destination.s('Trait: ' + trait + '\n')
				destination.write(config + '-' + gaitname + '\n')	
				traitPreexists[idx] = 1			
			idx = idx + 1

		# Write last two lines of library
		destination.write('\n' )
		destination.write('# LEAVE THIS LINE HERE (one enter line below last trait list) for correct parsing')

		# Close the file.
		source.close()
		destination.close()
		

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

			counter = self.counter

			rungait(self, ref_angles = self.ref_angles)

			# Simulation Step
			self.space.collide((), self._nearcb)
			self.world.step(1/self.fps)
			self._cjoints.empty()
			self.render()

			# Limit the FPS.
			self.clock.tick(self.fps)
			self.counter = self.counter + 1
		

# Main method for standalone mode.
if (__name__ == '__main__'):
	"""
	Instantiates a simulator in gait creator mode.
	"""

	print info

	obstaclefile = None
	robotfile = "config/" + sys.argv[1] + ".ckbot"
	if len(sys.argv)==3:
            obstaclefile = "obstacles/" + sys.argv[2] + ".obstacle"

	instance = GaitCreator(robotfile, standalone=1, obstaclefile=obstaclefile)
	instance.run()
