#!/usr/bin/env python
"""
===================================================================
heatController.py - Potential Field Region-to-Region Motion Control
===================================================================

Uses the heat-controller to take a current position, current region, and destination region and return a global velocity vector that will help us get there
"""

import __heatControllerHelper as heatControllerHelper
from numpy import *
from __is_inside import is_inside
import time

import lib.handlers.handlerTemplates as handlerTemplates

class HeatControllerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data):
        """
        Heat motion planning controller
        """
        self.drive_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler)
        self.pose_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler)
        self.fwd_coordmap = executor.hsub.coordmap_map2lab
        self.rfi = executor.proj.rfi
        self.last_warning = 0

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        If ``last`` is true, we will move to the center of the region.

        Returns ``True`` if we are outside the supposed ``current_reg``
        """

        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return True

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
            transFaceIdx = None
        else:
            # Find a face to go through
            # TODO: Account for non-determinacy?
            # For now, let's just choose the largest face available, because we are probably using a big clunky robot
            # TODO: Why don't we just store this as the index?
            transFaceIdx = None
            max_magsq = 0
            for i, face in enumerate(self.rfi.regions[current_reg].getFaces()):
                if face not in self.rfi.transitions[current_reg][next_reg]:
                    continue

                tf_pta, tf_ptb = face
                tf_vector = tf_ptb - tf_pta
                magsq = (tf_vector.x)**2 + (tf_vector.y)**2
                if magsq > max_magsq:
                    transFaceIdx = i
                    max_magsq = magsq

            if transFaceIdx is None:
                print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.rfi.regions[current_reg].name, self.rfi.regions[next_reg].name)

        # Transform the region vertices into real coordinates
        pointArray = [x for x in self.rfi.regions[current].getPoints()]
        pointArray = map(self.fwd_coordmap, pointArray)
        vertices = mat(pointArray).T

        # Get a controller function
        controller = heatControllerHelper.getController(vertices, transFaceIdx, last)

        # Cache it in
        cache[current][next] = controller

        return controller

