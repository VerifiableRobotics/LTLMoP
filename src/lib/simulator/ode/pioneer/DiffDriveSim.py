#!/usr/bin/env python

import ode, xode.parser
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math, time, copy, sys

info = """DiffDriveSim

Differential Drive Simulator for LTLMoP
[Sebastian Castro - Cornell University]
[Autonomous Systems Laboratory - 2011]

Manual Controls (Standalone Mode):
Move around with W A S D
Stop moving with X

"""

class DiffDriveSim:
	"""
	Differential Drive Robot Simulator Class
	"""

	fps = 25.0
	cameraDistance = 30.0
	vel = 0.5
	turn = 0.5
	turn2 = 0.5
	clip = 150.0
	res = (800, 600)

	def __init__(self,standalone=1,obstaclefile=None,regionfile=None,region_calib=None,startingpose=None):
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
			self.loadRegionData(regionfile)
			self.region_calib = region_calib

		# Simulation world parameters.
		self._initOpenGL()
		self.world = ode.World()
		self.world.setGravity((0, -9.81, 0))
		self.world.setERP(0.1)
		self.space = ode.Space()
		self.ground = ode.GeomPlane(space=self.space, normal=(0,1,0), dist=0)

		# Robot part parameters.
		self.cubesize = 6.0
		self.cubemass = 10.0
		self.wheelradius = 3.0
		self.wheelmass = 5.0
		self.track = 2.85*self.wheelradius
		self.hingemaxforce = 5000000

		# Initial Data
		self.basepos = [0, 0, 0]
		self.baserot = self.genmatrix(0,1)      	# Identity Matrix for now.
		self.counter = 0
		self.left_speed = 0.0
		self.right_speed = 0.0
		
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
		self.clicking = False
		
		# Load the obstacle file.
		if (obstaclefile!=None):
		    self.loadObstacles(obstaclefile)

	def _loadObjects(self):
		"""
		Spawns the Differential Drive robot based on the input pose.
		"""  
		
		# Create a Box
		box_pos = [self.basepos[0], self.basepos[1] + self.cubesize, self.basepos[2]]
		rot = self.baserot
				
		boxbody = ode.Body(self.world)
		boxgeom = ode.GeomBox(space=self.space, lengths=(self.cubesize, self.cubesize*0.75, self.cubesize) )
		boxgeom.setBody(boxbody)
		boxgeom.setPosition(box_pos)
		boxgeom.setRotation(rot)
		boxM = ode.Mass()
		boxM.setBox(self.cubemass,self.cubesize,self.cubesize*0.75,self.cubesize)
		boxbody.setMass(boxM)	
		
		# Create two Cylindrical Wheels
		rot = self.multmatrix(self.genmatrix(math.pi/2,2),rot)
		leftwheel_pos = [box_pos[0] + 0.5*self.track, box_pos[1] - self.wheelradius*0.5, box_pos[2] + self.cubesize*0.2]
		rightwheel_pos = [box_pos[0] - 0.5*self.track, box_pos[1] - self.wheelradius*0.5, box_pos[2]  + self.cubesize*0.2]
		
		leftwheelbody = ode.Body(self.world)
		leftwheelgeom = ode.GeomCylinder(space=self.space, radius = self.wheelradius, length = self.wheelradius/3.0 )
		leftwheelgeom.setBody(leftwheelbody)
		leftwheelgeom.setPosition(leftwheel_pos)
		leftwheelgeom.setRotation(rot)
		leftwheelM = ode.Mass()
		leftwheelM.setCappedCylinder(self.wheelmass, 1, self.wheelradius, self.wheelradius/3.0)
		leftwheelbody.setMass(leftwheelM)	
		
		lefthinge = ode.HingeJoint(self.world)
		lefthinge.attach(leftwheelbody,boxbody)
		lefthinge.setAnchor(leftwheel_pos)
		lefthinge.setAxis(self.rotate((0,0,1),rot))
		lefthinge.setParam(ode.ParamFMax,self.hingemaxforce)

		rightwheelbody = ode.Body(self.world)
		rightwheelgeom = ode.GeomCylinder(space=self.space, radius = self.wheelradius, length = self.wheelradius )
		rightwheelgeom.setBody(rightwheelbody)
		rightwheelgeom.setPosition(rightwheel_pos)
		rightwheelgeom.setRotation(rot)
		rightwheelM = ode.Mass()
		rightwheelM.setCappedCylinder(self.wheelmass, 1, self.wheelradius, self.cubemass/3.0)
		rightwheelbody.setMass(rightwheelM)	
		
		righthinge = ode.HingeJoint(self.world)
		righthinge.attach(rightwheelbody,boxbody)
		righthinge.setAnchor(rightwheel_pos)
		righthinge.setAxis(self.rotate((0,0,1),rot))
		righthinge.setParam(ode.ParamFMax,self.hingemaxforce)

		# Create a spherical wheel at the back for support
		caster_pos = [box_pos[0], box_pos[1] - self.cubesize*0.5, box_pos[2]  - self.cubesize*0.5]

		casterbody = ode.Body(self.world)
		castergeom = ode.GeomSphere(space=self.space, radius = self.cubesize/5.0)
		castergeom.setBody(casterbody)
		castergeom.setPosition(caster_pos)
		castergeom.setRotation(rot)
		casterM = ode.Mass()
		casterM.setSphere(self.cubemass*100.0, self.cubesize/5.0)
		casterbody.setMass(casterM)			
		
		self.fixed = ode.FixedJoint(self.world)
		self.fixed.attach(casterbody,boxbody)
		self.fixed.setFixed()
		
		# WHEW, THE END OF ALL THAT FINALLY!
		# Build the Geoms and Joints arrays for rendering.
		self.boxgeom = boxgeom
		self._geoms = [boxgeom, leftwheelgeom, rightwheelgeom, castergeom, self.ground]
		self.lefthinge = lefthinge
		self.righthinge = righthinge
		self._joints = [self.lefthinge, self.righthinge, self.fixed]
		

        def loadObstacles(self,obstaclefile):
            """
            Loads obstacles from the obstacle text file.
            """

            # Initiate data structures.
            data = open(obstaclefile,"r")
            obs_sizes = [];
            obs_positions = [];
            obs_masses = [];
            reading = "None"

            # Parse the obstacle text file.
            for line in data:
                linesplit = line.split()
                if linesplit != []:
                    if linesplit[0] != "#":
                        obs_sizes.append([float(linesplit[0]),float(linesplit[1]),float(linesplit[2])])
                        obs_positions.append([float(linesplit[3]),float(linesplit[4]),float(linesplit[5])])
                        obs_masses.append(float(linesplit[6]))      

            # Go through all the obstacles in the list and spawn them.
            for i in range(len(obs_sizes)):

                obs_size = obs_sizes[i]
                obs_pos = obs_positions[i]
                obs_mass = obs_masses[i]
                        
                # Create the obstacle.
                body = ode.Body(self.world)
                geom = ode.GeomBox(space=self.space, lengths=obs_size )
                geom.setBody(body)
                geom.setPosition(obs_pos)
                M = ode.Mass()
                M.setBox(obs_mass,obs_size[0],obs_size[1],obs_size[2])
                body.setMass(M)

                # Append all these new pointers to the simulator class.
                self._geoms.append(geom)
		
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
				# Do not plot the boundary region.
				elif info[0].lower()!="boundary":
					# Polygon-type region -- extract color and all the vertices in the polygon.
					if info[1]=="poly":
						region_color = [float(info[6])/255, float(info[7])/255, float(info[8])/255]
						posx = int(info[2])
						posy = int(info[3])
						vertices = []
						for idx in range(9,len(info),2):
							vertices.append([posx + int(info[idx]), posy + int(info[idx+1])])
						temp_info = region_color
						temp_info.extend(vertices)
						self.region_data.append(temp_info)
					elif info[1]=="rect":
						region_color = [float(info[6])/255, float(info[7])/255, float(info[8])/255]
						posx = int(info[2])
						posy = int(info[3])
						width = int(info[4])
						height = int(info[5])
						vertices = []
						vertices.append([posx, posy])
						vertices.append([posx, posy + height])
						vertices.append([posx + width, posy + height])
						vertices.append([posx + width, posy])
						temp_info = region_color
						temp_info.extend(vertices)
						self.region_data.append(temp_info)


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
			rot = (1, 0, 0, 0, math.cos(angle), -math.sin(angle), 0, math.sin(angle), math.cos(angle))
		elif (axis==2):	# Y-AXIS
			rot = (math.cos(angle), 0, math.sin(angle), 0, 1, 0, -math.sin(angle), 0, math.cos(angle))
		elif (axis==3): # Z-AXIS
			rot = (math.cos(angle), -math.sin(angle), 0, math.sin(angle), math.cos(angle), 0, 0 , 0, 1)

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
		screen = pygame.display.set_mode(self.res, pygame.OPENGL | pygame.DOUBLEBUF)
		pygame.display.set_caption('Pioneer Simulation')
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
		Render either a ode.GeomBox, ode.GeomCylinder or ode.GeomSphere object.
		"""
		
		allowed = [ode.GeomBox, ode.GeomCylinder, ode.GeomSphere]
		ok = False
		for klass in allowed:
			ok = ok or isinstance(geom, klass)
		if (not ok):
			return
				
		glPushMatrix()
		glMultMatrixd(self._extractMatrix(geom))

		if (isinstance(geom, ode.GeomBox)):
			color = (0.5, 0, 0)
		
		glMaterialfv(GL_FRONT, GL_SPECULAR, color)

		if (isinstance(geom, ode.GeomBox)):
			sx, sy, sz = geom.getLengths()
			glScale(sx, sy, sz)
			glutInit()
			glutSolidCube(1)
		elif (isinstance(geom, ode.GeomCylinder)):
			inner = self.wheelradius/2.5
			outer = self.wheelradius - inner
			glutSolidTorus( inner, outer, 50, 50 ) 		# Draw a torus since cylinders don't work.
		elif (isinstance(geom, ode.GeomSphere)):
			r = geom.getRadius()
			glutSolidSphere(r, 20, 20)

		glPopMatrix()
        

	def _renderGround(self):
		"""
		Renders the ground plane.
		"""

		normal, d = self.ground.getParams()
		x, y, z = self.boxgeom.getPosition()

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
			x, y, z = self.boxgeom.getPosition()
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
		if (key == pygame.K_l):
			self._align = 1.0
			
		# Drive the robot with WASD
		elif (key == pygame.K_w):
			self.left_speed = self.left_speed + 0.5
			self.right_speed = self.right_speed + 0.5
		elif (key == pygame.K_a):
			self.left_speed = self.left_speed - 0.25
			self.right_speed = self.right_speed + 0.25
		elif (key == pygame.K_s):
			self.left_speed = self.left_speed - 0.5
			self.right_speed = self.right_speed - 0.5
		elif (key == pygame.K_d):
			self.left_speed = self.left_speed + 0.25
			self.right_speed = self.right_speed - 0.25
		
		# Stop with X
		elif (key == pygame.K_x):
			self.left_speed = 0
			self.right_speed = 0
			
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
			c.setBounce(0.05)
			c.setMu(10000)
			j = ode.ContactJoint(self.world, self._cjoints, c)
			j.attach(body1, body2)


	def get2DPose(self):
		"""
		Get the 2D Pose (x, y, yaw) of the differential drive robot.
		"""

		# Find 2-D pose using robot body's position and rotation matrix.
		# NOTE: Due to the definition of the axes, the (x,y) position is actually (x,-z)
		pose2d = [0,0,0]
		# X and Y values are the average of the lower and upper parts of the robot body.
		pose2d[0] = self.boxgeom.getPosition()[0]
		pose2d[1] = -self.boxgeom.getPosition()[2]
		# Find 2D Orientation using the rotation matrix and the x-direction unit vector.
		# Rotate the vector [1, 0, 0] using the rotation matrix and find the angle to the x-axis using the atan2 function (to cover all 4 quadrants)
		rot = self.boxgeom.getRotation()
		rotvec = self.rotate([1,0,0],rot)		
		pose2d[2] = math.atan2(-rotvec[2],rotvec[0]) - math.pi/2

		return pose2d

	def setWheelSpeeds(self, left, right):
		"""
		Set the linear velocities of both wheels.
		"""
		
		self.left_speed = float(left)/self.wheelradius
		self.right_speed = float(right)/self.wheelradius
		
		
	def setVW(self, v, w):
		"""
		Set the linear and angular velocities of the robot.
		"""
		
		# From differential drive kinematics:
		left = v - 0.5*self.track*w
		right = v + 0.5*self.track*w
		self.setWheelSpeeds(left, right)
		
		
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
			
			# Set wheel hinge speeds.
			self.lefthinge.setParam(ode.ParamVel, self.left_speed)
			self.righthinge.setParam(ode.ParamVel, self.right_speed)
			
			#pose = self.get2DPose()
			#print pose
			
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

		# Set wheel hinge speeds.
		self.lefthinge.setParam(ode.ParamVel, self.left_speed)
		self.righthinge.setParam(ode.ParamVel, self.right_speed)

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
	Instantiates a simulator and runs it in stand-alone mode with all default arguments.
	"""

	print info

	obstaclefile = None
	if len(sys.argv)==2:
		obstaclefile = "obstacles/" + sys.argv[2] + ".obstacle"

	sim = DiffDriveSim(standalone=1, obstaclefile=obstaclefile)
	sim.run()
