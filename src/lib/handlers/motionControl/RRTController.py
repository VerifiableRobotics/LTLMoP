#!/usr/bin/env python
"""
===================================================================
RRTController.py - Rapidly-Exploring Random Trees Motion Controller
===================================================================

Uses the vector field algorithm developed by Stephen R. Lindemann to calculate a global velocity vector to take the robot from the current region to the next region, through a specified exit face.
"""

import _RRTControllerHelper
from numpy import *
from __is_inside import *
import Polygon
import time, math
import sys,os

class motionControlHandler:
    def __init__(self, proj, shared_data,robot_type,max_angle,plotting):
        """
        Rapidly-Exploring Random Trees alogorithm motion planning controller

        robot_type (int): Which robot is used for execution. Nao is 1, STAGE is 2,ODE is 3(default=3)
        max_angle (float): The difference in angle between the two nodes allowed. The value should be between 0 to 6.28 = 2*pi (default=1.047)
        plotting (int): Enable plotting is 1 and disable plotting is 0 (default=1)
        """

        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.h_instance['drive']
        self.pose_handler = proj.h_instance['pose']

        # Get information about regions
        self.proj = proj
        #self.proj.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab
        self.coordmap_lab2map = proj.coordmap_lab2map
        self.last_warning = 0
        self.previous_next_reg = None

        # Store the Rapidly-Exploring Random Tress Built
        self.RRT_V = None               # array containing all the points on the RRT Tree
        self.RRT_E = None               # array specifying the connection of points on the Tree
        self.RRT_V_toPass = None
        self.RRT_E_toPass = None
        self.point_to_go = None
        self.heading     = None
        self.E_prev      = None
        self.Velocity    = None
        self.currentRegionPoly = None
        self.nextRegionPoly    = None
        self.map               = {}
        self.all               = Polygon.Polygon()

        # Information about the robot
        if robot_type not in [1,2,3]:
            robot_type = 1
        self.system = robot_type

        if max_angle > 2*pi:
            max_angle = 2*pi
        if max_angle < 0:
            max_angle = 0
        self.max_angle_allowed = max_angle
        self.plotting          = plotting

        ## 1: Nao ; 2: STAGE; 3: 0DE
        if  self.system == 1:
            self.radius = 0.15*1.2
        elif self.system == 2:
            self.radius = 0.1
        elif self.system == 3:
            self.radius = 5

        for region in self.proj.rfi.regions:
            self.map[region.name] = self.createRegionPolygon(region)
            for n in range(len(region.holeList)): # no of holes
                self.map[region.name] -= self.createRegionPolygon(region,n)

        for regionName,regionPoly in self.map.iteritems():
            self.all += regionPoly

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
            self.nextRegionPoly    = self.map[self.proj.rfi.regions[next_reg].name]
            self.currentRegionPoly = self.map[self.proj.rfi.regions[current_reg].name]

            print "next Region is "+str(self.proj.rfi.regions[next_reg].name)
            print "Current Region is "+str(self.proj.rfi.regions[current_reg].name)

            pointArray = [x for x in self.proj.rfi.regions[current_reg].getPoints()]
            pointArray = map(self.coordmap_map2lab, pointArray)
            vertices = mat(pointArray).T

            #set to zero velocity before tree is generated
            self.drive_handler.setVelocity(0, 0)
            if last:
                transFace = None
            else:
                # Find a face to go through
                # TODO: Account for non-determinacy?

                transFace = None
                # Find the index of this face
                q_gBundle = [[],[]]

                for i in range(len(self.proj.rfi.transitions[current_reg][next_reg])):
                    pointArray_transface = [x for x in self.proj.rfi.transitions[current_reg][next_reg][i]]
                    transFace = asarray(map(self.coordmap_map2lab,pointArray_transface))
                    bundle_x = (transFace[0,0] +transFace[1,0])/2    #mid-point coordinate x
                    bundle_y = (transFace[0,1] +transFace[1,1])/2    #mid-point coordinate y
                    q_gBundle = hstack((q_gBundle,vstack((bundle_x,bundle_y))))

                if transFace is None:
                    print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.proj.rfi.regions[current_reg].name, self.proj.rfi.regions[next_reg].name)

            # Run algorithm to build the Rapid-Exploring Random Trees
            self.RRT_V = None
            self.RRT_E = None
            self.RRT_V,self.RRT_E,self.heading,self.E_prev,self.RRT_V_toPass,self.RRT_E_toPass = _RRTControllerHelper.buildTree([pose[0], pose[1]],pose[2], vertices,\
                self.radius,self.system,self.currentRegionPoly, self.nextRegionPoly,q_gBundle,self.map,self.all,self.max_angle_allowed, self.plotting)

            # map the lab coordinates back to pixels
            V_tosend = array(mat(self.RRT_V[1:,:])).T
            V_tosend = map(self.coordmap_lab2map, V_tosend)
            V_tosend = mat(V_tosend).T
            s = 'RRT:E'+"["+str(list(self.RRT_E[0]))+","+str(list(self.RRT_E[1]))+"]"+':V'+"["+str(list(self.RRT_V[0]))+","+str(list(V_tosend[0]))+","+str(list(V_tosend[1]))+"]"+':T'+"["+str(list(q_gBundle[0]))+","+str(list(q_gBundle[1]))+"]"

            print s

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        V = _RRTControllerHelper.setVelocity([pose[0], pose[1]], self.RRT_V,self.RRT_E,self.heading,self.E_prev,self.radius)
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
            for r in self.proj.rfi.regions:
                pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                vertices = mat(pointArray).T

                if is_inside([pose[0], pose[1]], vertices):
                    #print "I think I'm in " + r.name
                    #print pose
                    break
            self.last_warning = time.time()

        return arrived

    def createRegionPolygon(self,region,hole = None):
        """
        This function takes in the region points and make it a Polygon.
        """
        if hole == None:
            pointArray = [x for x in region.getPoints()]
        else:
            pointArray = [x for x in region.getPoints(hole_id = hole)]
        pointArray = map(self.coordmap_map2lab, pointArray)
        regionPoints = [(pt[0],pt[1]) for pt in pointArray]
        formedPolygon= Polygon.Polygon(regionPoints)
        return formedPolygon
