#!/usr/bin/env python
"""
===================================================================
RRTController.py - Rapidly-Exploring Random Trees Motion Controller
===================================================================

Uses the vector field algorithm developed by Stephen R. Lindemann to calculate a global velocity vector to take the robot from the current region to the next region, through a specified exit face.
"""

import RRTControllerHelper
from numpy import *
from is_inside import *
import Polygon
import time, math
import sys,os

class motionControlHandler:
    def __init__(self, proj, shared_data):
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.drive_handler
        self.pose_handler = proj.pose_handler

        # Get information about regions
        self.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab
        self.coordmap_lab2map = proj.coordmap_lab2map
        self.last_warning = 0
        self.previous_next_reg = None

        # Store the Rapidly-Exploring Random Tress Built
        self.RRT_V = None
        self.RRT_E = None
        self.RRT_V_toPass = None
        self.RRT_E_toPass = None
        self.point_to_go = None
        self.heading     = None
        self.E_prev      = None
        self.Velocity    = None
        self.currentRegionPoly = None
        self.nextRegionPoly    = None

        # Information about the robot
        self.system = 1
        ## 1: Nao ; 2: STAGE; 3: 0DE
        if  self.system == 1:
            self.radius = 0.15*1.2
        elif self.system == 2:
            self.radius = 0.1
        elif self.system == 3:
            self.radius = 5

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

        ###This part is run when the robot goes to a new region, otherwise, the original tree will be used.
        if not self.previous_next_reg == current_reg:

            # NOTE: Information about region geometry can be found in self.rfi.regions
            # if the current region is "freespace", it will remove the other regions from the polygon boundary
            pointArray = [x for x in self.rfi.regions[next_reg].getPoints()]
            pointArray = map(self.coordmap_map2lab, pointArray)
            regionPoints = [(pt[0],pt[1]) for pt in pointArray]
            #print "regionPoints:", regionPoints
            self.nextRegionPoly = Polygon.Polygon(regionPoints)
            if self.rfi.regions[next_reg].name.lower()=='freespace':
                for region in self.rfi.regions:
                    if region.name.lower() != 'freespace':
                        pointArray = [x for x in region.getPoints()]
                        pointArray = map(self.coordmap_map2lab, pointArray)
                        regionPoints = [(pt[0],pt[1]) for pt in pointArray]
                        self.nextRegionPoly = self.nextRegionPoly - Polygon.Polygon(regionPoints)

            pointArray = [x for x in self.rfi.regions[current_reg].getPoints()]
            #print "pointArray",pointArray,len(pointArray)
            pointArray = map(self.coordmap_map2lab, pointArray)
            regionPoints = [(pt[0],pt[1]) for pt in pointArray]
            print "regionPoints:", regionPoints
            self.currentRegionPoly = Polygon.Polygon(regionPoints)
            if self.rfi.regions[current_reg].name.lower()=='freespace':
                for region in self.rfi.regions:
                    if region.name.lower() != 'freespace':
                        pointArray = [x for x in region.getPoints()]
                        pointArray = map(self.coordmap_map2lab, pointArray)
                        regionPoints = [(pt[0],pt[1]) for pt in pointArray]
                        self.currentRegionPoly = self.currentRegionPoly - Polygon.Polygon(regionPoints)

            vertices = mat(pointArray).T

            #set to zero velocity before tree is generated
            self.drive_handler.setVelocity(0, 0)
            if last:
                transFace = None
            else:
                print "110 hello why"

                # Find a face to go through
                # TODO: Account for non-determinacy?

                transFace = None
                # Find the index of this face
                # TODO: Why don't we just store this as the index?
                ##########just trying here
                q_gBundle = [[],[]]
                if self.rfi.regions[current_reg].name.lower()=='freespace':
                    isFreespace = True
                else:
                    isFreespace = False
                #print "isFreespace", isFreespace

                #print "size of array(self.rfi.transitions[current_reg][next_reg])", self.rfi.transitions[current_reg][next_reg],len(self.rfi.transitions[current_reg][next_reg])

                for i in range(len(self.rfi.transitions[current_reg][next_reg])):
                    pointArray_transface = [x for x in self.rfi.transitions[current_reg][next_reg][i]]
                    #print  pointArray_transface
                    transFace = asarray(map(self.coordmap_map2lab,pointArray_transface))
                    #print "RRT Controller 138: transface" ,transFace
                    bundle_x = (transFace[0,0] +transFace[1,0])/2    #mid-point coordinate x
                    bundle_y = (transFace[0,1] +transFace[1,1])/2    #mid-point coordinate y
                    q_gBundle = hstack((q_gBundle,vstack((bundle_x,bundle_y))))
                print "RRT Controller 140: q_gbundle", q_gBundle


                if transFace is None:
                    print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.rfi.regions[current_reg].name, self.rfi.regions[next_reg].name)

            # Run algorithm to build the Rapid-Exploring Random Trees
            print "149: ok you pass"

            ################ WORKS SILLL HERE
            self.RRT_V = None
            self.RRT_E = None
            self.RRT_V,self.RRT_E,self.heading,self.E_prev,self.RRT_V_toPass,self.RRT_E_toPass = RRTControllerHelper.buildTree([pose[0], pose[1]],pose[2], vertices, self.radius,self.system,self.currentRegionPoly,self.nextRegionPoly,isFreespace,q_gBundle)

            # map the lab coordinates back to pixels

            V_tosend = array(mat(self.RRT_V[1:,:])).T
            V_tosend = map(self.coordmap_lab2map, V_tosend)
            V_tosend = mat(V_tosend).T
            s = 'RRT:E'+"["+str(list(self.RRT_E[0]))+","+str(list(self.RRT_E[1]))+"]"+':V'+"["+str(list(self.RRT_V[0]))+","+str(list(V_tosend[0]))+","+str(list(V_tosend[1]))+"]"+':T'+"["+str(list(q_gBundle[0]))+","+str(list(q_gBundle[1]))+"]"
            """
            V_tosend = array(mat(self.RRT_V_toPass[1:,:])).T
            V_tosend = map(self.coordmap_lab2map, V_tosend)
            V_tosend = mat(V_tosend).T
            s = 'RRT:E'+"["+str(list(self.RRT_E_toPass[0]))+","+str(list(self.RRT_E_toPass[1]))+"]"+':V'+"["+str(list(self.RRT_V_toPass[0]))+","+str(list(V_tosend[0]))+","+str(list(V_tosend[1]))+"]"
            """
            print s

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        V = RRTControllerHelper.setVelocity([pose[0], pose[1]], self.RRT_V,self.RRT_E,self.heading,self.E_prev,self.radius)
        self.Velocity = V[0:2,0]
        self.heading = V[2,0]
        self.E_prev = V[3,0]
        self.previous_next_reg = current_reg

        # Pass this desired velocity on to the drive handler
        self.drive_handler.setVelocity(V[0,0], V[1,0], pose[2])
        RobotPoly = Polygon.Shapes.Circle(self.radius,(pose[0],pose[1]))
        #step 4: check whether robot is inside the boundary

        departed = not self.currentRegionPoly.covers(RobotPoly)
        arrived  = self.nextRegionPoly.covers(RobotPoly)


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
