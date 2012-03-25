#!/usr/bin/env python
"""
=======================================================
vectorController.py - Vector Addition Motion Controller
=======================================================

Uses the vector field algorithm developed by Stephen R. Lindemann to calculate a global velocity vector to take the robot from the current region to the next region, through a specified exit face.
"""

import vectorControllerHelper
from numpy import *
from is_inside import *
import time, math

class motionControlHandler:
    def __init__(self, proj, shared_data):
        """
        Vector motion planning controller
        """

        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.h_instance['drive']
        self.pose_handler = proj.h_instance['pose']
        
        # Get information about regions
        self.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab
        self.last_warning = 0

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.

        Returns ``True`` if we've reached the destination region.
        """

        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return False

        # Find our current configuration
        pose = self.pose_handler.getPose()

        # Check if Vicon has cut out
        # TODO: this should probably go in posehandler?
        if math.isnan(pose[2]):
            print "WARNING: No Vicon data! Pausing."
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            time.sleep(1)
            return False

        # NOTE: Information about region geometry can be found in self.rfi.regions:
        pointArray = [x for x in self.rfi.regions[current_reg].getPoints()]
        pointArray = map(self.coordmap_map2lab, pointArray)
        vertices = mat(pointArray).T 

        if last:
            transFace = None
        else:
            # Find a face to go through
            # TODO: Account for non-determinacy?
            # For now, let's just choose the largest face available, because we are probably using a big clunky robot
            max_magsq = 0
            for tf in self.rfi.transitions[current_reg][next_reg]:
                magsq = (tf[0].x - tf[1].x)**2 + (tf[0].y - tf[1].y)**2
                if magsq > max_magsq:
                    pt1, pt2 = tf
                    max_magsq = magsq
            
            transFace = None
            # Find the index of this face
            # TODO: Why don't we just store this as the index?
            for i, face in enumerate([x for x in self.rfi.regions[current_reg].getFaces()]):
                # Account for both face orientations...
                if (pt1 == face[0] and pt2 == face[1]) or (pt1 == face[1] and pt2 == face[0]):
                    transFace = i
                    break
            
            if transFace is None:
                print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.rfi.regions[current_reg].name, self.rfi.regions[next_reg].name)
         

		# Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        V = vectorControllerHelper.getController([pose[0], pose[1]], vertices, transFace)

        # Pass this desired velocity on to the drive handler
        self.drive_handler.setVelocity(V[0], V[1], pose[2])
        
        departed = not is_inside([pose[0], pose[1]], vertices)
        pointArray = [x for x in self.rfi.regions[next_reg].getPoints()]
        pointArray = map(self.coordmap_map2lab, pointArray)
        vertices = mat(pointArray).T 
        # Figure out whether we've reached the destination region
        arrived = is_inside([pose[0], pose[1]], vertices)

        if departed and (not arrived) and (time.time()-self.last_warning) > 0.5:
            #print "WARNING: Left current region but not in expected destination region"
            # Figure out what region we think we stumbled into
            for r in self.rfi.regions:
                pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                vertices = mat(pointArray).T 

                if is_inside([pose[0], pose[1]], vertices):
                    #print "I think I'm in " + r.name
                    #print pose
                    break
            self.last_warning = time.time()

        return arrived
