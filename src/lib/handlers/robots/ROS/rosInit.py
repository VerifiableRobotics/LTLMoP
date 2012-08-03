#!/usr/bin/env python
"""
=================================================
rosSim.py - ROS/Gazebo Initialization Handler
=================================================
"""
import math
import roslib; roslib.load_manifest('gazebo')
import sys, subprocess, os, time, os, shutil, rospy
import cairo, rsvg
import re, Polygon, Polygon.IO
import lib.regions as regions
#import execute
from numpy import *
#from gazebo.srv import *
from gazebo_msgs.srv import *
from gazebo import gazebo_interface
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import fileinput

class initHandler:
	def __init__(self, proj, init_region, useLTLMoPWorld=True, robotPixelWidth=200, robotPhysicalWidth=.5,package="pr2_gazebo",launchFile="pr2.launch", calib=False):
		"""
		Initialization handler for ROS and gazebo

		init_region (region): The initial region of the robot
		useLTLMoPWorld (bool): Whether you want to use a blank world with the LTLMoP map or a predifined world (default=True)
		robotPixelWidth (int): The width of the robot in pixels in ltlmop (default=200)
		robotPhysicalWidth (float): The physical width of the robot in meters (default=.5)
		package (str): The package where the robot is located (default="pr2_gazebo")
		launchFile (str): The launch file name for the robot in the package (default="pr2.launch")
		"""

		self.offset=[0,0]
		if useLTLMoPWorld:
			#Create a box in gazebo to represent the region map floor
			self.createRegionMapInGazebo()
		
			#Change robot in gazebo:
			self.changeRobot()

			# Create a subprocess for ROS
			self.createROSProcess(package='gazebo_worlds',launchFile='ltlmop.launch')

			if not calib :
 				# Move the region map so the robot is 'centered' 
				self.moveRegionMap()
		else:
			pass	
			
		#The following is the global node for ltlmop handlers
		#Nodes are like processes
		rospy.init_node('LTLMoPHandlers')

	def getSharedData(self):
		# TODO: Return a dictionary of any objects that will need to be shared with other handlers
		return {'ROS_INIT_HANDLER': self}

	def region2svg(self, proj, regionFile):
		"""
		Converts region file to svg
		This is from the deprecated regions file with slight changes for 
		proper calculation of the size of the regions map
		"""
		fout=re.sub(r"\.region$", ".svg", regionFile)
		rfi = regions.RegionFileInterface()
		rfi.readFile(regionFile)

		polyList = []
		
		for region in rfi.regions:
			points = [(pt.x,-pt.y) for pt in region.getPoints()]
			poly = Polygon.Polygon(points)
			polyList.append(poly)
		try: #Normal Operation
			boundary=proj.rfiold.regions[proj.rfiold.indexOfRegionWithName("boundary")]
		except: #Calibration
			boundary=proj.rfi.regions[proj.rfi.indexOfRegionWithName("boundary")]
		width=boundary.size.width
		height=boundary.size.height

		#use boundary size for image size
		Polygon.IO.writeSVG(fout, polyList, width=width,height=height)

		return fout #return the file name

	def replaceAll(self,file,searchExp,replaceExp):
		"""
		Relaces lines in a file given a search string and replacement string
		You only need a portion of the line to match the searchExp and 
		then it will replace the whole line with the replaceExp
		"""
		for line in fileinput.input(file, inplace=1):
			if searchExp in line:
				line = line.replace(line,replaceExp)
			sys.stdout.write(line)

	def createRegionMapInGazebo(self):
		"""
		The following creates a box object in Gazebo as the floor
		This box object loads a texture element that is a png of
		the regions file that is used in ltlmop
		Essentially this loads the ltlmop map beneath the robot
		"""
		#The package and launch file for the robot that is being used		
		self.package=package
		self.launchFile=launchFile
		#Map to real world scaling constant
		self.ratio=robotPhysicalWidth/robotPixelWidth

		#The next few blocks creates a png copy of the regions to load into gazebo
		#This block creates a copy and converts to svg
		texture_dir =  '/opt/ros/fuerte/stacks/simulator_gazebo/gazebo/gazebo/share/gazebo-1.0.2/Media/materials/textures' #potentially dangerous as pathd in ROS change with updates
		ltlmop_path = proj.getFilenamePrefix()
		regionsFile = ltlmop_path+"_copy.regions"
		shutil.copy(proj.rfi.filename,regionsFile)
		svgFile = self.region2svg(proj, regionsFile)
		svg = rsvg.Handle(file=svgFile)
	
		#This block converts the svg to png and applies naming conventions
		self.imgWidth=svg.props.width
		self.imgHeight=svg.props.height
		img = cairo.ImageSurface(cairo.FORMAT_ARGB32, self.imgWidth, self.imgHeight)
		ctx = cairo.Context(img)
		handler = rsvg.Handle(svgFile)
		handler.render_cairo(ctx)
		img.write_to_png(ltlmop_path+"_simbg.png")
		ltlmop_map_path = ltlmop_path + "_simbg.png"
		shutil.copy (ltlmop_map_path, texture_dir)
		full_pic_path = texture_dir + "/" + proj.project_basename + "_simbg.png"
		shutil.copyfile (full_pic_path, (texture_dir + "/" + 'ltlmop_map.png'))

		# Change size of region map in gazebo
		# This is accomplished through edits of the world file before opening
		path="/opt/ros/fuerte/stacks/simulator_gazebo/gazebo_worlds/worlds/ltlmop_map.world" # Potential problem when version changes >_<
		searchExp='<box size='
		T=[self.ratio*self.imgWidth,self.ratio*self.imgHeight]
		resizeX=T[0]
		resizeY=T[1]
		replaceExp='            <box size="'+str(resizeX)+' '+str(resizeY)+' .1" />\n'
		self.replaceAll(path,searchExp,replaceExp)

	def changeRobot(self):
		"""
		This changes the robot in the ltlmop launch file
		It cannot be used with other world files
		"""
		#Accomplish through edits in the launch file
		path="/opt/ros/fuerte/stacks/simulator_gazebo/gazebo_worlds/launch/ltlmop.launch"
		searchExp='<include file='
		replaceExp='    <include file="$(find '+self.package+')/launch/'+self.launchFile+'" />\n'
		self.replaceAll(path,searchExp,replaceExp)

	def createROSProcess(self, package, launchFile):
		"""
		This creates a process for ROS and pipes output to the terminal
		"""
		start = subprocess.Popen(['roslaunch', package, launchFile], stdout=subprocess.PIPE)
		start_output = start.stdout
		
		# Wait for it to fully start up
		while 1:
			input = start_output.readline()
			print input, # Pass it on
			if input == '': # EOF
				print "(INIT) WARNING:  Gazebo seems to have died!"
				break
			if "Successfully spawned" in input:
			#Successfully spawend is output from the creation of the PR2
			#It might get stuck waiting for another type of robot to spawn
				time.sleep(5)
				break

	def moveRegionMap(self):
		# Start in the center of the defined initial region
		try: #Normal operation
			initial_region = proj.rfiold.regions[proj.rfiold.indexOfRegionWithName(init_region)]
		except: #Calibration
			initial_region = proj.rfi.regions[proj.rfi.indexOfRegionWithName(init_region)]
		center = initial_region.getCenter()

		# Load the map calibration data and the region file data to feed to the simulator
		coordmap_map2lab,coordmap_lab2map = proj.getCoordMaps()
		map2lab = list(coordmap_map2lab(array(center)))

		print "Initial region name: ", initial_region.name, " I think I am here: ", map2lab, " and center is: ", center

		#get the current pose
		rospy.wait_for_service('/gazebo/get_model_state')
		cms = ModelState()
		try:
			gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			cms = gms('plane1_model','world')
		except rospy.ServiceException, e:
			print 'Service Call Failed: %s'%e

		#create a new pose
		model_state = ModelState()
		model_state.model_name = 'plane1_model'
		model_state.pose = cms.pose
		#Teleporting the robot is very dangerous! So insted, we move
		#the floor beneath the robot.    
		model_state.pose.position.x = -map2lab[0]
		model_state.pose.position.y = -map2lab[1]

		#Storage for pose correction		
		self.offset=(map2lab[0],map2lab[1])

		#send the pose to the map
		rospy.wait_for_service('/gazebo/set_model_state')
		try:
			sms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
			sms(model_state)	
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
