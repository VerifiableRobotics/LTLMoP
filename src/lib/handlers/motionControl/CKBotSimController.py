#!/usr/bin/env python
"""
==================================================
CKBotSimController.py - Simulated CKBot Controller
==================================================
"""

import math
from numpy import *

class motionControlHandler:
	def __init__(self, proj, shared_data):
		# Get references to handlers we'll need to communicate with.
		self.drive_handler = proj.drive_handler
		self.pose_handler = proj.pose_handler
		
		# Other useful definitions.
		self.vx = 0
		self.vy = 0
		self.w = 0
		self.mode = "transition"
        
		# Get information about regions.
		self.rfi = proj.rfi
		self.coordmap_lab2map = proj.coordmap_lab2map

	def gotoRegion(self, current_reg, next_reg, last=False):
		"""
		Transition to the next region as dictated by the FSA.
		Once a transition has been made, first move to the center of the next region and then transition further.
		This makes incidence angles to region transition faces much more benign (i.e. closer to normal).
		"""

		if current_reg == next_reg and not last:
			# No need to move!
			self.drive_handler.setVelocity(0, 0)  # So let's stop
			return True

		# If we are transitioning to another region then move towards the midpoint of the transition face.
		elif self.mode == "transition":

		    # Find our current configuration.
			pose = self.pose_handler.getPose()
			pose_map = list(self.coordmap_lab2map(array([pose[0],pose[1]])))

			# Find the midpoint of the transition face by finding the two common points to the current and next regions.
			curArray = [x for x in self.rfi.regions[current_reg].getPoints()]
			nextArray = [x for x in self.rfi.regions[next_reg].getPoints()]
			transpoints = []
			for cur in curArray:
				for nex in nextArray:
					if (nex[0]==cur[0] and nex[1]==cur[1]):
						transpoints.append(cur)
			targetpoint = [0.5*(transpoints[0][0]+transpoints[1][0]),0.5*(transpoints[0][1]+transpoints[1][1])]

			# Find an angle between the robot's position and the transition face's position.
			dx = targetpoint[0]-pose_map[0]
			dy = -(targetpoint[1]-pose_map[1])			
			angle = math.atan2(dy,dx)
			#print angle*180.0/math.pi
			
			# If we are too close to the boundary then simply keep moving in the same direction (avoids jitter)
			dist = math.sqrt( math.pow(dx,2) + math.pow(dy,2) )
			if dist > 25:
				[self.vx, self.vy, self.w] = [math.cos(angle), math.sin(angle), 0]
				
			# Send the velocity command to the drive handler.
			self.drive_handler.setVelocity(self.vx, self.vy, self.w)
		    
			# Figure out whether we've reached the destination region.
			# If we have reached the destination region then switch to center mode (i.e. transition to the center of the next region before making the next transition)
			arrived = not self.rfi.regions[current_reg].objectContainsPoint(pose_map[0],pose_map[1])
			if arrived:
				self.mode = "center"
			return arrived
			
		# If we are trying to go to the center of the current region, go here.	
		elif self.mode == "center":
		
		    # Find our current configuration.
			pose = self.pose_handler.getPose()
			pose_map = list(self.coordmap_lab2map(array([pose[0],pose[1]])))
			
			# Find the center of the current region.
			targetpoint = self.rfi.regions[current_reg].getCenter()

			# Find an angle between the robot's position and the transition face's position.
			dx = targetpoint[0]-pose_map[0]
			dy = -(targetpoint[1]-pose_map[1])			
			angle = math.atan2(dy,dx)
			#print angle*180.0/math.pi
			
			# If we are close enough to the center of the region then switch to transition mode.
			dist = math.sqrt( math.pow(dx,2) + math.pow(dy,2) )
			if dist < 15:
				self.mode = "transition"
			
			# Send the velocity command to the drive handler.
			[self.vx, self.vy, self.w] = [math.cos(angle), math.sin(angle), 0]
			self.drive_handler.setVelocity(self.vx, self.vy, self.w)
			
			return False
