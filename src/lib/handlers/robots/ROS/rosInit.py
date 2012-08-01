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
	def __init__(self, proj, init_region, robotPixelWidth=200, robotPhysicalWidth=.5, calib=False):
		"""
		Initialization handler for ROS and gazebo

		init_region (region): The initial region of the robot
		robotPixelWidth (int): The width of the robot in pixels in ltlmop (default=200)
		robotPhysicalWidth (float): The physical width of the robot in meters (default=.5)
		"""
		
		#Map to real world scaling constant
		self.ratio=robotPhysicalWidth/robotPixelWidth

		#The next few blocks creates a png copy of the regions to load into gazebo
		#This block creates a copy and converts to svg
		texture_dir =  '/opt/ros/fuerte/stacks/simulator_gazebo/gazebo/gazebo/share/gazebo-1.0.2/Media/materials/textures'
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

		# Change size of region map in gazebo:
		# Potential problem when version changes >_<
		path="/opt/ros/fuerte/stacks/simulator_gazebo/gazebo_worlds/worlds/ltlmop_map.world"
		searchExp='<box size='
		T=array([[self.ratio*self.imgWidth,0],[0,self.ratio*self.imgHeight]])
		resizeX=T[0][0]
		resizeY=T[1][1]
		replaceExp='            <box size="'+str(resizeX)+' '+str(resizeY)+' .1" />\n'
		self.replaceAll(path,searchExp,replaceExp)

		# Create a subprocess
		start = subprocess.Popen(['roslaunch', 'gazebo_worlds', 'ltlmop.launch'], stdout=subprocess.PIPE)
		start_output = start.stdout
		
		# Wait for it to fully start up
		while 1:
			input = start_output.readline()
			print input, # Pass it on
			if input == '': # EOF
				print "(INIT) WARNING:  Gazebo seems to have died!"
				break
			if "Successfully spawned" in input:
				time.sleep(5)
				break

		# Move the region map so the robot is 'centered' 
		if not calib :
			# Start in the center of the defined initial region
			try:
				initial_region = proj.rfiold.regions[proj.rfiold.indexOfRegionWithName(init_region)]
			except:
				initial_region = proj.rfi.regions[proj.rfi.indexOfRegionWithName(init_region)]
			center = initial_region.getCenter()

			# Load the map calibration data and the region file data to feed to the simulator
			#coordmap_map2lab,coordmap_lab2map = proj.getCoordMaps()
			#map2lab = list(coordmap_map2lab(array(center)))
			map2lab=(self.ratio*(center[0]-self.imgWidth/2),-self.ratio*(center[1]-self.imgHeight/2))

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
			pr2_state=ModelState()
			pr2_state.model_name='pr2'
			pr2_state.pose.orientation.z=-pi/2

			#send the pose to the map
			rospy.wait_for_service('/gazebo/set_model_state')
			try:
				sms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
				sms(model_state)	
				#sms(pr2_state)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

	def getSharedData(self):
		# TODO: Return a dictionary of any objects that will need to be shared with other handlers
		return {'ROS_INIT_HANDLER': self}

	def region2svg(self, proj, regionFile):
		"""
		Converts region file to svg
		This is from the deprecated regions file with slight changes
		"""
		fout=re.sub(r"\.region$", ".svg", regionFile)
		rfi = regions.RegionFileInterface()
		rfi.readFile(regionFile)

		polyList = []
		
		width=0
		height=0

		for region in rfi.regions:
			points = [(pt.x,-pt.y) for pt in region.getPoints()]
			poly = Polygon.Polygon(points)
			polyList.append(poly)
		try:
			boundary=proj.rfiold.regions[proj.rfiold.indexOfRegionWithName("boundary")]
		except:
			boundary=proj.rfi.regions[proj.rfi.indexOfRegionWithName("boundary")]
		width=boundary.size.width
		height=boundary.size.height

		#use boundary size for image size
		Polygon.IO.writeSVG(fout, polyList, width=width,height=height)

		return fout

	def replaceAll(self,file,searchExp,replaceExp):
		"""
		Relaces lines in a file given a search string and replacement string
		"""
		for line in fileinput.input(file, inplace=1):
			if searchExp in line:
				line = line.replace(line,replaceExp)
			sys.stdout.write(line)

