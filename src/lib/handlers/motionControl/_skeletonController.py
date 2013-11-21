#!/usr/bin/env python
"""
==================================================
skeletonController.py - Skeleton Motion Controller
==================================================
"""

from numpy import *

class motionControlHandler:
    def __init__(self, proj, shared_data):
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.drive_handler
        self.pose_handler = proj.pose_handler
        
        # Get information about regions
        self.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab

    def gotoRegion(self, current_reg, next_reg):
        """
        Returns ``True`` if we've reached the destination region.
        """

        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return True  # Already there

        # Find our current configuration
        pose = self.pose_handler.getPose()

        # NOTE: Information about region geometry can be found in self.rfi.regions:
        pointArray = [x for x in self.rfi.regions[current_reg].getPoints()]
        pointArray = map(self.coordmap_map2lab, pointArray)
        vertices = mat(pointArray).T 

        # TODO: Calculate a velocity vector in the *GLOBAL REFERENCE FRAME* 
        # that will get us on our way to the next region
        vx, vy = 0, 0

        # Pass this desired velocity on to the drive handler
        # It also gets the current heading of the robot
        self.drive_handler.setVelocity(vx, vy, pose[2])
        
        # Figure out whether we've exited the current region
        if is_inside([pose[0], pose[1]], vertices):
            arrived = False
        else:
            arrived = True

        return arrived
