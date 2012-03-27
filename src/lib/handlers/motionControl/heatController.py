#!/usr/bin/env python
"""
===================================================================
heatController.py - Potential Field Region-to-Region Motion Control
===================================================================

Uses the heat-controller to take a current position, current region, and destination region and return a global velocity vector that will help us get there
"""

import heatControllerHelper
from numpy import *
from is_inside import *
import time

class motionControlHandler:
    def __init__(self, proj, shared_data):
        """
        Heat motion planning controller
        """
        self.drive_handler = proj.drive_handler
        self.pose_handler = proj.pose_handler
        self.fwd_coordmap = proj.coordmap_map2lab
        self.rfi = proj.rfi
        self.last_warning = 0

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        If ``last`` is true, we will move to the center of the region.
        
        Returns ``True`` if we are outside the supposed ``current_reg``
        """

        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return False

        controller = self.get_controller(current_reg, next_reg, last)

        pose = self.pose_handler.getPose()

        [X, DqX, F, inside, J] = controller(mat(pose[0:2]).T)

        self.drive_handler.setVelocity(X[0,0], X[1,0], pose[2])
        
        # Transform the region vertices into real coordinates
        pointArray = [self.fwd_coordmap(x) for x in self.rfi.regions[next_reg].getPoints()]
        vertices = mat(pointArray).T 

        # Figure out whether we've reached the destination region
        if is_inside([pose[0], pose[1]], vertices):
            arrived = True
        else:
            arrived = False

        if (arrived != (not inside)) and (time.time()-self.last_warning) > 0.5:
            print "WARNING: Left current region but not in expected destination region"
            # Figure out what region we think we stumbled into
            for r in self.rfi.regions:
                pointArray = [self.fwd_coordmap(x) for x in r.getPoints()]
                vertices = mat(pointArray).T 

                if is_inside([pose[0], pose[1]], vertices):
                    print "I think I'm in " + r.name
                    print pose
                    break
            self.last_warning = time.time()

        return arrived


    def get_controller(self, current, next, last, cache={}):
        """
        Wrapper for the controller factory, with caching.
        """

        # Check to see if we already have an appropriate controller stored in the cache.
        # TODO: Account for last in cache

        if current in cache and next in cache[current]:
            return cache[current][next]

        # If not, create a space in the cache to put our new controller.

        cache[current] = {}
        
        # Let's go get a controller!

        if last:
            transFace = None
        else:
            # Find a face to go through
            # TODO: Account for non-determinacy?
            # For now, let's just choose the largest face available, because we are probably using a big clunky robot
            max_magsq = 0
            for tf in self.rfi.transitions[current][next]:
                magsq = (tf[0].x - tf[1].x)**2 + (tf[0].y - tf[1].y)**2
                if magsq > max_magsq:
                    pt1, pt2 = tf
                    max_magsq = magsq
            
            transFace = None
            # Find the index of this face
            # TODO: Why don't we just store this as the index?
            for i, face in enumerate([x for x in self.rfi.regions[current].getFaces()]):
                # Account for both face orientations...
                if (pt1 == face[0] and pt2 == face[1]) or (pt1 == face[1] and pt2 == face[0]):
                    transFace = i
                    break
            
            if transFace is None:
                print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.rfi.regions[current].name, self.rfi.regions[next].name)
         
        # Transform the region vertices into real coordinates
        pointArray = [x for x in self.rfi.regions[current].getPoints()]
        pointArray = map(self.fwd_coordmap, pointArray)
        vertices = mat(pointArray).T 
        
        # Get a controller function
        controller = heatControllerHelper.getController(vertices, transFace, last)

        # Cache it in
        cache[current][next] = controller

        return controller

