#!/usr/bin/env python
"""
===================================================
CKBot.py -- CKBot Simulation Initialization Handler
===================================================
"""

import os, sys, time
import CKBotSim
from numpy import *

sys.path.append('home/cornell/Downloads/ltlmop-asl/handlers/init')
import CKBotSim

class initHandler:
    def __init__(self, proj, calib=False):

	# IN PROGRESS: Spawn the object in the center of a region.
	# Can currently select the name of the starting region and spawn the robot there.
	# What we would like to achieve is to read the initial region from the simulator itself.
	region_data = proj.rfi.regions
	region_name = "deck"
	for elem in region_data:
	    if elem.name == region_name:
		center = elem.getCenter()

	# Load the map calibration data and the region file data to feed to the simulator
        exp_cfg_name = proj.spec_data['SETTINGS']['currentExperimentName'][0]
        exp_cfg_data = proj.getExperimentConfig(exp_cfg_name)
        coordmap_map2lab,coordmap_lab2map = proj.getCoordMaps(exp_cfg_data)
	region_calib = list(coordmap_map2lab(array([1,1])))
	regionfile = os.path.join(proj.project_root, proj.spec_data['SETTINGS']['RegionFile'][0])

	# Multiply the initial pose by the calibration constants from specEditor.
	initial_pose = [region_calib[0]*center[0], region_calib[1]*center[1]]
	# Convert this initial (x,y) pose to (x,y,z) pose in the simulator.
	# The convenction is (x,y) => (x,0,-y) as the simulator's y is normal to the ground.
	initial_pose_sim = [initial_pose[0], 0, -initial_pose[1]]

	# Initiate the CKBot simulator and render it once.
	self.simulator = CKBotSim.CKBotSim("robots/CKBot/Snake.ckbot",standalone=0, regionfile = regionfile,region_calib = region_calib, startingpose=initial_pose_sim)
	self.simulator.run_once()

    def getSharedData(self):
        # Return a dictionary of any objects that will need to be shared with
        # other handlers

        return {'Simulator': self.simulator}
