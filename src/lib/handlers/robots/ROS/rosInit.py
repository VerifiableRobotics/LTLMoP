#!/usr/bin/env python
"""
=================================================
rosSim.py - ROS/Gazebo Initialization Handler
=================================================
"""
import roslib; roslib.load_manifest('gazebo')
import sys, subprocess, os, time, os, shutil, rospy
import cairo, rsvg
import re, Polygon, Polygon.IO
import lib.regions as regions
#import execute
from numpy import *
from gazebo.srv import *

class initHandler:
    def __init__(self, proj, init_region, calib=False):
	"""
	Initialization handler for ROS and gazebo

	init_region (region): The initial region of the robot
	"""

	texture_dir =  '/opt/ros/fuerte/stacks/simulator_gazebo/gazebo/gazebo/share/gazebo-1.0.2/Media/materials/textures'
	ltlmop_path = proj.getFilenamePrefix()
	regionsFile = proj.rfiold.filename#ltlmop_path + ".regions"
	svgFile = self.region2svg(regionsFile)
	svg = rsvg.Handle(file=svgFile)
	
	img = cairo.ImageSurface(cairo.FORMAT_ARGB32, svg.props.width, svg.props.height)
	ctx = cairo.Context(img)
	handler = rsvg.Handle(svgFile)
	handler.render_cairo(ctx)
	img.write_to_png(ltlmop_path+"_simbg.png")
	
	ltlmop_map_path = ltlmop_path + "_simbg.png"

	shutil.copy (ltlmop_map_path, texture_dir)
	full_pic_path = texture_dir + "/" + proj.project_basename + "_simbg.png"
	shutil.copyfile (full_pic_path, (texture_dir + "/" + 'ltlmop_map.png'))

	
	# Set ROS package path to include gazebo_worlds where the launch files are.
	#root = proj.ltlmop_root
	#temp = os.path.join(root, 'etc')
	#temp = os.path.join(temp,'gazebo_worlds')
	#ROS_PACKAGE_PATH = temp + ":" + os.getenv('ROS_PACKAGE_PATH')
	#os.environ["ROS_PACKAGE_PATH"] = ROS_PACKAGE_PATH

	
	# Create a subprocess
	start = subprocess.Popen(['roslaunch', 'gazebo_worlds', 'ltlmop.launch'], stdout=subprocess.PIPE)
	#start = subprocess.Popen(['roslaunch', 'ltlmop_ros', 'iros.launch'], stdout=subprocess.PIPE)
        start_output = start.stdout
	# Wait for it to fully start up

	while 1:
		input = start_output.readline()
	        print input, # Pass it on
	        if input == '': # EOF
	                print "(INIT) WARNING:  Gazebo seems to have died!"
	                break
		#if "spawning success True" in input:	
		if ("Successfully spawned" or "successfully spawned") in input:
	                time.sleep(1)
	                break
  	
	if not calib :
		# Start in the center of the defined initial region
		#initial_region = proj.rfiold.regions[int(proj.exp_cfg_data['InitialRegion'][0])]
		initial_region = proj.rfi.regions[proj.rfi.indexOfRegionWithName(init_region)]
		center = initial_region.getCenter()

		# Load the map calibration data and the region file data to feed to the simulator
		coordmap_map2lab,coordmap_lab2map = proj.getCoordMaps()#proj.exp_cfg_data)
		map2lab = list(coordmap_map2lab(array(center)))

		print "Initial region name: ", initial_region.name, " I think I am here: ", map2lab, " and center is: ", center
		#print proj.exp_cfg_data['Calibration'][0]

		model_state = gazebo.msg.ModelState()
		model_state.model_name = 'pr2'
		model_state.pose.position.x = map2lab[0]
		model_state.pose.position.y = map2lab[1]		

		rospy.wait_for_service('/gazebo/set_model_state')
	    	try:
			sms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
			sms(model_state)	
		except rospy.ServiceException, e:
	       		print "Service call failed: %s"%e

	
    def getSharedData(self):
        # TODO: Return a dictionary of any objects that will need to be shared with
        # other handlers
        return {'ROS_INIT_HANDLER': self}

    def region2svg(self, regionFile):
	fout=re.sub(r"\.region$", ".svg", regionFile)
	rfi = regions.RegionFileInterface()
	rfi.readFile(regionFile)

	polyList = []

	for region in rfi.regions:
	    points = [(pt.x,pt.y) for pt in region.getPoints()]
	    poly = Polygon.Polygon(points)
	    polyList.append(poly)

	Polygon.IO.writeSVG(fout, polyList)

	return fout
