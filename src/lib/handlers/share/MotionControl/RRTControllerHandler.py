#!/usr/bin/env python
"""
===================================================================
RRTController.py - Rapidly-Exploring Random Trees Motion Controller
===================================================================

Uses Rapidly-exploring Random Tree Algorithm to generate paths given the starting position and the goal point.
"""

from numpy import *
from __is_inside import *
import math
import sys,os
from scipy.linalg import norm
from numpy.matlib import zeros
import __is_inside
import time, sys,os
import scipy as Sci
import scipy.linalg
import Polygon, Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes
from math import sqrt, fabs , pi
import random
import thread
import threading

# importing matplotlib to show the path if possible
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import_matplotlib = True
except:
    print "matplotlib is not imported. Plotting is disabled"
    import_matplotlib = False

import lib.handlers.handlerTemplates as handlerTemplates

class RRTControllerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data,robot_type,max_angle_goal,max_angle_overlap,plotting):
        """
        Rapidly-Exploring Random Trees alogorithm motion planning controller

        robot_type (int): Which robot is used for execution. BasicSim is 1, ODE is 2, ROS is 3, Nao is 4, Pioneer is 5(default=1)
        max_angle_goal (float): The biggest difference in angle between the new node and the goal point that is acceptable. If it is bigger than the max_angle, the new node will not be connected to the goal point. The value should be within 0 to 6.28 = 2*pi. Default set to 6.28 = 2*pi (default=6.28)
        max_angle_overlap (float): difference in angle allowed for two nodes overlapping each other. If you don't want any node overlapping with each other, put in 2*pi = 6.28. Default set to 1.57 = pi/2 (default=1.57)
        plotting (bool): Check the box to enable plotting. Uncheck to disable plotting (default=True)
        """

        self.system_print       = False       # for debugging. print on GUI ( a bunch of stuffs)
        self.finish_print       = False       # set to 1 to print the original finished E and V before trimming the tree
        self.orientation_print  = False        # show the orientation information of the robot

        # Get references to handlers we'll need to communicate with
        self.drive_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler)
        self.pose_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler)

        # Get information about regions
        self.proj              = executor.proj
        self.coordmap_map2lab  = executor.hsub.coordmap_map2lab
        self.coordmap_lab2map  = executor.hsub.coordmap_lab2map
        self.last_warning      = 0
        self.previous_next_reg = None

        # Store the Rapidly-Exploring Random Tress Built
        self.RRT_V              = None               # array containing all the points on the RRT Tree
        self.RRT_E              = None               # array specifying the connection of points on the Tree
        self.E_current_column   = None               # the current column on the tree (to find the current heading point)
        self.Velocity           = None
        self.currentRegionPoly  = None
        self.nextRegionPoly     = None
        self.map                = {}
        self.all                = Polygon.Polygon()
        self.trans_matrix       = mat([[0,1],[-1,0]])   # transformation matrix for find the normal to the vector
        self.stuck_thres        = 20          # threshold for changing the range of sampling omega

        # Information about the robot (default set to ODE)
        if robot_type not in [1,2,3,4,5]:
            robot_type = 1
        self.system = robot_type

        # Information about maximum turning angle allowed from the latest node to the goal point
        if max_angle_goal > 2*pi:
            max_angle_goal = 2*pi
        if max_angle_goal < 0:
            max_angle_goal = 0
        self.max_angle_allowed = max_angle_goal

        # Information about maximum difference in angle allowed between two overlapping nodes
        if max_angle_overlap > 2*pi:
            max_angle_overlap = 2*pi
        if max_angle_overlap < 0:
            max_angle_overlap = 0
        self.max_angle_overlap = max_angle_overlap


        # Information about whether plotting is enabled.
        if plotting is True and import_matplotlib == True:
            self.plotting          = True
        else:
            self.plotting          = False

        # Specify the size of the robot
        # 1: basicSim; 2: ODE; 3: ROS  4: Nao; 5: Pioneer
        #  self.radius: radius of the robot
        #  self.timestep  : number of linear segments to break the curve into for calculation of x, y position
        #  self.step_size  : the length of each step for connection to goal point
        #  self.velocity   : Velocity of the robot in m/s in control space (m/s)
        if self.system  == 1:
            self.radius = 5
            self.step_size  = 25
            self.timeStep = 10
            self.velocity = 2    # 1.5
        if  self.system == 2:
            self.radius = 5
            self.step_size  = 15
            self.timeStep = 10
            self.velocity = 2    # 1.5
        elif self.system == 3:
            self.ROSInitHandler = shared_data['ROS_INIT_HANDLER']
            self.radius = self.ROSInitHandler.robotPhysicalWidth/2
            self.step_size  = self.radius*3 #0.2
            self.timeStep = 8
            self.velocity  = self.radius/2  #0.08
        elif self.system == 4:
            self.radius = 0.15*1.2
            self.step_size  = 0.2      #set the step_size for points be 1/5 of the norm  ORIGINAL = 0.4
            self.timeStep = 5
            self.velocity  = 0.05
        elif self.system == 5:
            self.radius = 0.15
            self.step_size  = 0.2      #set the step_size for points be 1/5 of the norm  ORIGINAL = 0.4
            self.timeStep = 5
            self.velocity  = 0.05


        # Operate_system (int): Which operating system is used for execution.
        # Ubuntu and Mac is 1, Windows is 2
        if sys.platform in ['win32', 'cygwin']:
            self.operate_system = 2
        else:
            self.operate_system = 1

        if self.system_print == True:
            print "The operate_system is "+ str(self.operate_system)

        # Generate polygon for regions in the map
        for region in self.proj.rfi.regions:
            self.map[region.name] = self.createRegionPolygon(region)
            for n in range(len(region.holeList)): # no of holes
                self.map[region.name] -= self.createRegionPolygon(region,n)

        # Generate the boundary polygon
        for regionName,regionPoly in self.map.iteritems():
            self.all += regionPoly

        # Start plotting if operating in Windows
        if self.operate_system == 2 and self.plotting ==True:
            # start using anmination to plot the robot
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111)
            self.scope = _Scope(self.ax,self)
            thread.start_new_thread(self.jplot,())

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.
        Returns ``True`` if we've reached the destination region.
        """

        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return True

        # Find our current configuration
        pose = self.pose_handler.getPose()

        # Check if Vicon has cut out
        # TODO: this should probably go in posehandler?
        if math.isnan(pose[2]):
            print "WARNING: No Vicon data! Pausing."
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            time.sleep(1)
            return False

        ###This part will be run when the robot goes to a new region, otherwise, the original tree will be used.
        if not self.previous_next_reg == next_reg:
            # Entered a new region. New tree should be formed.
            self.nextRegionPoly    = self.map[self.proj.rfi.regions[next_reg].name]
            self.currentRegionPoly = self.map[self.proj.rfi.regions[current_reg].name]

            if self.system_print == True:
                print "next Region is " + str(self.proj.rfi.regions[next_reg].name)
                print "Current Region is " + str(self.proj.rfi.regions[current_reg].name)

            #set to zero velocity before tree is generated
            self.drive_handler.setVelocity(0, 0)
            if last:
                transFace = None
            else:
                # Determine the mid points on the faces connecting to the next region (one goal point will be picked among all the mid points later in buildTree)
                transFace   = None
                q_gBundle   = [[],[]] # list of goal points (midpoints of transition faces)
                face_normal = [[],[]] # normal of the trnasition faces
                for i in range(len(self.proj.rfi.transitions[current_reg][next_reg])):
                    pointArray_transface = [x for x in self.proj.rfi.transitions[current_reg][next_reg][i]]
                    transFace = asarray(map(self.coordmap_map2lab,pointArray_transface))
                    bundle_x = (transFace[0,0] +transFace[1,0])/2    #mid-point coordinate x
                    bundle_y = (transFace[0,1] +transFace[1,1])/2    #mid-point coordinate y
                    q_gBundle     = hstack((q_gBundle,vstack((bundle_x,bundle_y))))

                    #find the normal vector to the face
                    face          = transFace[0,:] - transFace[1,:]
                    distance_face = norm(face)
                    normal        = face/distance_face * self.trans_matrix
                    face_normal   = hstack((face_normal,vstack((normal[0,0],normal[0,1]))))


                if transFace is None:
                    print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.proj.rfi.regions[current_reg].name, self.proj.rfi.regions[next_reg].name)

            # Run algorithm to build the Rapid-Exploring Random Trees
            self.RRT_V = None
            self.RRT_E = None

            # For plotting
            if self.operate_system == 2:
                if self.plotting == True:
                    self.ax.cla()
                else:
                    self.ax = None
            else:
                self.ax = None

            if self.operate_system == 1 and self.plotting == True:
                plt.cla()
                self.plotMap(self.map)
                plt.plot(pose[0],pose[1],'ko')

            self.RRT_V,self.RRT_E,self.E_current_column = self.buildTree(\
            [pose[0], pose[1]],pose[2],self.currentRegionPoly, self.nextRegionPoly,q_gBundle,face_normal)

            """
            # map the lab coordinates back to pixels
            V_tosend = array(mat(self.RRT_V[1:,:])).T
            V_tosend = map(self.coordmap_lab2map, V_tosend)
            V_tosend = mat(V_tosend).T
            s = 'RRT:E'+"["+str(list(self.RRT_E[0]))+","+str(list(self.RRT_E[1]))+"]"+':V'+"["+str(list(self.RRT_V[0]))+","+str(list(V_tosend[0]))+","+str(list(V_tosend[1]))+"]"+':T'+"["+str(list(q_gBundle[0]))+","+str(list(q_gBundle[1]))+"]"
            #print s
            """

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        self.Velocity = self.getVelocity([pose[0], pose[1]], self.RRT_V,self.RRT_E)
        #self.Node = self.getNode([pose[0], pose[1]], self.RRT_V,self.RRT_E)
        self.previous_next_reg = next_reg

        # Pass this desired velocity on to the drive handler
        self.drive_handler.setVelocity(self.Velocity[0,0], self.Velocity[1,0], pose[2])
        #self.drive_handler.setVelocity(self.Node[0,0], self.Node[1,0], pose[2])
        RobotPoly = Polygon.Shapes.Circle(self.radius,(pose[0],pose[1]))

        # check if robot is inside the current region
        departed = not self.currentRegionPoly.overlaps(RobotPoly)
        arrived  = self.nextRegionPoly.covers(RobotPoly)

        if departed and (not arrived) and (time.time()-self.last_warning) > 0.5:
            # Figure out what region we think we stumbled into
            for r in self.proj.rfi.regions:
                pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                vertices = mat(pointArray).T

                if is_inside([pose[0], pose[1]], vertices):
                    print "I think I'm in " + r.name
                    print pose
                    break
            self.last_warning = time.time()

        #print "arrived:"+str(arrived)
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

    def getVelocity(self,p, V, E, last=False):
        """
        This function calculates the velocity for the robot with RRT.
        The inputs are (given in order):
            p        = the current x-y position of the robot
            E        = edges of the tree  (2 x No. of nodes on the tree)
            V        = points of the tree (2 x No. of vertices)
            last = True, if the current region is the last region
                 = False, if the current region is NOT the last region
        """

        pose     = mat(p).T

        #dis_cur = distance between current position and the next point
        dis_cur  = vstack((V[1,E[1,self.E_current_column]],V[2,E[1,self.E_current_column]]))- pose

        heading = E[1,self.E_current_column]        # index of the current heading point on the tree
        if norm(dis_cur) < 1.5*self.radius:         # go to next point
            if not heading == shape(V)[1]-1:
                self.E_current_column = self.E_current_column + 1
                dis_cur  = vstack((V[1,E[1,self.E_current_column]],V[2,E[1,self.E_current_column]]))- pose
            #else:
            #    dis_cur  = vstack((V[1,E[1,self.E_current_column]],V[2,E[1,self.E_current_column]]))- vstack((V[1,E[0,self.E_current_column]],V[2,E[0,self.E_current_column]]))

        Vel = zeros([2,1])
        Vel[0:2,0] = dis_cur/norm(dis_cur)*0.5                    #TUNE THE SPEED LATER
        return Vel

    def getNode(self,p, V, E, last=False):
        """

        This function calculates the velocity for the robot with RRT.
        The inputs are (given in order):
            p        = the current x-y position of the robot

            E        = edges of the tree  (2 x No. of nodes on the tree)
            V        = points of the tree (2 x No. of vertices)
            last = True, if the current region is the last region
                 = False, if the current region is NOT the last region

        """

        pose     = mat(p).T

        #dis_cur = distance between current position and the next point
        dis_cur  = vstack((V[1,E[1,self.E_current_column]],V[2,E[1,self.E_current_column]]))- pose

        heading = E[1,self.E_current_column]        # index of the current heading point on the tree
        if norm(dis_cur) < 1.5*self.radius:         # go to next point
            if not heading == shape(V)[1]-1:
                self.E_current_column = self.E_current_column + 1
                dis_cur  = vstack((V[1,E[1,self.E_current_column]],V[2,E[1,self.E_current_column]]))- pose


        Node = zeros([2,1])
        Node[0,0] = V[1,E[1,self.E_current_column]]
        Node[1,0] = V[2,E[1,self.E_current_column]]
        #Vel[0:2,0] = dis_cur/norm(dis_cur)*0.5                    #TUNE THE SPEED LATER
        return Node


    def buildTree(self,p,theta,regionPoly,nextRegionPoly,q_gBundle,face_normal, last=False):
        """
        This function builds the RRT tree.
        p                : x,y position of the robot
        theta            : current orientation of the robot
        regionPoly       : current region polygon
        nextRegionPoly   : next region polygon
        q_gBundle        : coordinates of q_goals that the robot can reach
        face_normal      : the normal vector of each face corresponding to each goal point in q_gBundle
        """

        q_init          = mat(p).T
        V               = vstack((0,q_init))
        theta           = self.orientation_bound(theta)
        V_theta         = array([theta])

        #!!! CONTROL SPACE: generate a list of omega for random sampling
        omegaLowerBound = -math.pi/20      # upper bound for the value of omega
        omegaUpperBound = math.pi/20       # lower bound for the value of omega
        omegaNoOfSteps  = 20
        self.omega_range = linspace(omegaLowerBound,omegaUpperBound,omegaNoOfSteps)
        self.omega_range_escape = linspace(omegaLowerBound*4,omegaUpperBound*4,omegaNoOfSteps*4)    # range used when stuck > stuck_thres

        regionPolyOld = Polygon.Polygon(regionPoly)
        regionPoly += PolyShapes.Circle(self.radius*2.5,(q_init[0,0],q_init[1,0]))

        # check faces of the current region for goal points
        E     = [[],[]]       # the tree matrix
        Other = [[],[]]
        path                        = False          # if path formed then = 1
        stuck                       = 0              # count for changing the range of sampling omega
        append_after_latest_node    = False       # append new nodes to the latest node

        if self.system_print == True:
            print "plotting in buildTree is " + str(self.plotting)
        if self.plotting == True:
            if not plt.isinteractive():
                plt.ion()
            plt.hold(True)


        while not path:
            #step -1: try connection to q_goal (generate path to goal)
            i = 0

            if self.system_print == True:
                print "Try Connection to the goal points"

            # pushing possible q_goals into the current region (ensure path is covered by the current region polygon)
            q_pass = [[],[],[]]
            q_pass_dist = []
            q_gBundle = mat(q_gBundle)
            face_normal = mat(face_normal)

            while i < q_gBundle.shape[1]:
                q_g_original = q_gBundle[:,i]
                q_g = q_gBundle[:,i]+face_normal[:,i]*1.5*self.radius    ##original 2*self.radius
                #q_g = q_gBundle[:,i]+(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])*1.5*self.radius    ##original 2*self.radius
                if not regionPolyOld.isInside(q_g[0],q_g[1]):
                    #q_g = q_gBundle[:,i]-(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])*1.5*self.radius    ##original 2*self.radius
                    q_g = q_gBundle[:,i]-face_normal[:,i]*1.5*self.radius    ##original 2*self.radius

                #forming polygon for path checking
                EdgePolyGoal = PolyShapes.Circle(self.radius,(q_g[0,0],q_g[1,0])) + PolyShapes.Circle(self.radius,(V[1,shape(V)[1]-1],V[2:,shape(V)[1]-1]))
                EdgePolyGoal = PolyUtils.convexHull(EdgePolyGoal)
                dist = norm(q_g - V[1:,shape(V)[1]-1])

                #check connection to goal
                connect_goal = regionPoly.covers(EdgePolyGoal)   #check coverage of path from new point to goal

                # compare orientation difference
                thetaPrev = V_theta[shape(V)[1]-1]

                theta_orientation = abs(arctan((q_g[1,0]- V[2,shape(V)[1]-1])/(q_g[0,0]- V[1,shape(V)[1]-1])))
                if q_g[1,0] > V[2,shape(V)[1]-1]:
                    if q_g[0,0] < V[1,shape(V)[1]-1]: # second quadrant
                        theta_orientation = pi - theta_orientation
                    elif q_g[0,0] > V[1,shape(V)[1]-1]: # first quadrant
                        theta_orientation = theta_orientation
                elif q_g[1,0] < V[2,shape(V)[1]-1]:
                    if q_g[0,0] < V[1,shape(V)[1]-1]: #third quadrant
                        theta_orientation = pi + theta_orientation
                    elif q_g[0,0] > V[1,shape(V)[1]-1]: # foruth quadrant
                        theta_orientation =  2*pi - theta_orientation

                # check the angle between vector(new goal to goal_original ) and vector( latest node to new goal)
                Goal_to_GoalOriginal = q_g_original - q_g
                LatestNode_to_Goal   = q_g - V[1:,shape(V)[1]-1]
                Angle_Goal_LatestNode= arccos(vdot(array(Goal_to_GoalOriginal), array(LatestNode_to_Goal))/norm(Goal_to_GoalOriginal)/norm(LatestNode_to_Goal))

                # if connection to goal can be established and the max change in orientation of the robot is smaller than max_angle, tree is said to be completed.
                if self.orientation_print == True:
                    print "theta_orientation is " + str(theta_orientation)
                    print "thetaPrev is " + str(thetaPrev)
                    print "(theta_orientation - thetaPrev) is " + str(abs(theta_orientation - thetaPrev))
                    print "self.max_angle_allowed is " + str(self.max_angle_allowed)
                    print "abs(theta_orientation - thetaPrev) < self.max_angle_allowed" + str(abs(theta_orientation - thetaPrev) < self.max_angle_allowed)
                    print"Goal_to_GoalOriginal: " + str( array(Goal_to_GoalOriginal)) + "; LatestNode_to_Goal: " + str( array(LatestNode_to_Goal))
                    print vdot(array(Goal_to_GoalOriginal), array(LatestNode_to_Goal))
                    print "Angle_Goal_LatestNode" + str(Angle_Goal_LatestNode)

                if connect_goal and (abs(theta_orientation - thetaPrev) < self.max_angle_allowed) and (Angle_Goal_LatestNode < self.max_angle_allowed):

                    path = True
                    q_pass = hstack((q_pass,vstack((i,q_g))))
                    q_pass_dist = hstack((q_pass_dist,dist))

                i = i + 1

            if self.system_print == True:
                print "checked goal points"

            self.E = E
            self.V = V
            # connection to goal has established
            # Obtain the closest goal point that path can be formed.
            if path:
                if shape(q_pass_dist)[0] == 1:
                    cols = 0
                else:
                    (cols,) = nonzero(q_pass_dist == min(q_pass_dist))
                    cols = asarray(cols)[0]
                q_g = q_pass[1:,cols]
                """
                q_g = q_g-(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])*3*self.radius   #org 3
                if not nextRegionPoly.isInside(q_g[0],q_g[1]):
                    q_g = q_g+(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])*6*self.radius   #org 3
                """
                if self.plotting == True :
                    if self.operate_system == 1:
                        plt.suptitle('Rapidly-exploring Random Tree', fontsize=12)
                        plt.xlabel('x')
                        plt.ylabel('y')
                        if shape(V)[1] <= 2:
                            plt.plot(( V[1,shape(V)[1]-1],q_g[0,0]),( V[2,shape(V)[1]-1],q_g[1,0]),'b')
                        else:
                            plt.plot(( V[1,E[0,shape(E)[1]-1]], V[1,shape(V)[1]-1],q_g[0,0]),( V[2,E[0,shape(E)[1]-1]], V[2,shape(V)[1]-1],q_g[1,0]),'b')
                        plt.plot(q_g[0,0],q_g[1,0],'ko')
                        plt.figure(1).canvas.draw()
                    else:
                        BoundPolyPoints = asarray(PolyUtils.pointList(regionPoly))
                        self.ax.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
                        if shape(V)[1] <= 2:
                            self.ax.plot(( V[1,shape(V)[1]-1],q_g[0,0]),( V[2,shape(V)[1]-1],q_g[1,0]),'b')
                        else:
                            self.ax.plot(( V[1,E[0,shape(E)[1]-1]], V[1,shape(V)[1]-1],q_g[0,0]),( V[2,E[0,shape(E)[1]-1]], V[2,shape(V)[1]-1],q_g[1,0]),'b')
                        self.ax.plot(q_g[0,0],q_g[1,0],'ko')

                # trim the path connecting current node to goal point into pieces if the path is too long now
                numOfPoint = floor(norm(V[1:,shape(V)[1]-1]- q_g)/self.step_size)
                if numOfPoint < 3:
                    numOfPoint = 3
                x = linspace( V[1,shape(V)[1]-1], q_g[0,0], numOfPoint )
                y = linspace( V[2,shape(V)[1]-1], q_g[1,0], numOfPoint )
                for i in range(x.shape[0]):
                    if i != 0:
                        V = hstack((V,vstack((shape(V)[1],x[i],y[i]))))
                        E = hstack((E,vstack((shape(V)[1]-2,shape(V)[1]-1))))

                #push the goal point to the next region
                q_g = q_g+face_normal[:,q_pass[0,cols]]*3*self.radius    ##original 2*self.radius
                if not nextRegionPoly.isInside(q_g[0],q_g[1]):
                    q_g = q_g-face_normal[:,q_pass[0,cols]]*6*self.radius    ##original 2*self.radius
                V = hstack((V,vstack((shape(V)[1],q_g[0,0],q_g[1,0]))))
                E = hstack((E,vstack((shape(V)[1]-2 ,shape(V)[1]-1))))

                if self.plotting == True :
                    if self.operate_system == 1:
                        plt.plot(q_g[0,0],q_g[1,0],'ko')
                        plt.plot(( V[1,shape(V)[1]-1],V[1,shape(V)[1]-2]),( V[2,shape(V)[1]-1],V[2,shape(V)[1]-2]),'b')
                        plt.figure(1).canvas.draw()
                    else:
                        self.ax.plot(q_g[0,0],q_g[1,0],'ko')
                        self.ax.plot(( V[1,shape(V)[1]-1],V[1,shape(V)[1]-2]),( V[2,shape(V)[1]-1],V[2,shape(V)[1]-2]),'b')

            # path is not formed, try to append points onto the tree
            if not path:

                # connection_to_tree : connection to the tree is successful
                if append_after_latest_node:
                    V,V_theta,E,Other,stuck,append_after_latest_node, connection_to_tree = self.generateNewNode(V,V_theta,E,Other,regionPoly,stuck, append_after_latest_node)
                else:
                    connection_to_tree = False
                    while not connection_to_tree:
                        V,V_theta,E,Other,stuck,append_after_latest_node, connection_to_tree = self.generateNewNode  (V,V_theta,E,Other,regionPoly,stuck)


        if self.finish_print:
            print 'Here is the V matrix:', V, 'Here is the E matrix:',E
            print >>sys.__stdout__, 'Here is the V matrix:\n', V, '\nHere is the E matrix:\n',E

        #B: trim to a single path
        single = 0
        while single == 0:
            trim  = 0
            for j in range(shape(V)[1]-3):
                (row,col) = nonzero(E == j+1)
                if len(col) == 1:
                    E = delete(E, col[0], 1)
                    trim = 1

            if trim == 0:
                single = 1;

        ####print with matlib
        if self.plotting ==True :
            if self.operate_system == 1:
                plt.plot(V[1,:],V[2,:],'b')
                for i in range(shape(E)[1]):
                    plt.text(V[1,E[0,i]],V[2,E[0,i]], V[0,E[0,i]], fontsize=12)
                    plt.text(V[1,E[1,i]],V[2,E[1,i]], V[0,E[1,i]], fontsize=12)
                plt.figure(1).canvas.draw()
            else:
                BoundPolyPoints = asarray(PolyUtils.pointList(regionPoly))
                self.ax.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
                self.ax.plot(V[1,:],V[2,:],'b')
                for i in range(shape(E)[1]):
                    self.ax.text(V[1,E[0,i]],V[2,E[0,i]], V[0,E[0,i]], fontsize=12)
                    self.ax.text(V[1,E[1,i]],V[2,E[1,i]], V[0,E[1,i]], fontsize=12)

        #return V, E, and the current node number on the tree
        V = array(V)
        return V, E, 0


    def generateNewNode(self,V,V_theta,E,Other,regionPoly,stuck,append_after_latest_node =False):
        """
        Generate a new node on the current tree matrix
        V         : the node matrix
        V_theta   : the orientation matrix
        E         : the tree matrix (or edge matrix)
        Other     : the matrix containing the velocity and angular velocity(omega) information
        regionPoly: the polygon of current region
        stuck     : count on the number of times failed to generate new node
        append_after_latest_node : append new nodes to the latest node (True only if the previous node addition is successful)
        """


        if self.system_print == True:
            print "In control space generating path,stuck = " + str(stuck)

        connection_to_tree = False   # True when connection to the tree is successful

        if stuck > self.stuck_thres:
            # increase the range of omega since path cannot ge generated
            omega = random.choice(self.omega_range_escape)
        else:
            #!!!! CONTROL SPACE STEP 1 - generate random omega
            omega = random.choice(self.omega_range)


        #!!!! CONTROL SPACE STEP 2 - pick a random point on the tree
        if append_after_latest_node:
            tree_index = shape(V)[1]-1
        else:
            if random.choice([1,2]) == 1:
                tree_index = random.choice(array(V[0])[0])
            else:
                tree_index = shape(V)[1]-1


        xPrev     = V[1,tree_index]
        yPrev     = V[2,tree_index]
        thetaPrev = V_theta[tree_index]

        j = 1
        #!!!! CONTROL SPACE STEP 3 - Check path of the robot
        path_robot = PolyShapes.Circle(self.radius,(xPrev,yPrev))
        while j <= self.timeStep:
            xOrg      = xPrev
            yOrg      = yPrev
            xPrev     = xPrev + self.velocity/omega*(sin(omega* 1 + thetaPrev)-sin(thetaPrev))
            yPrev     = yPrev - self.velocity/omega*(cos(omega* 1 + thetaPrev)-cos(thetaPrev))
            thetaPrev = omega* 1 + thetaPrev
            path_robot = path_robot + PolyShapes.Circle(self.radius,(xPrev,yPrev))

            j = j + 1

        thetaPrev = self.orientation_bound(thetaPrev)
        path_all = PolyUtils.convexHull(path_robot)
        in_bound = regionPoly.covers(path_all)
        """
        # plotting
        if plotting == True:
            self.plotPoly(path_all,'r',1)
        """

        stuck = stuck + 1

        if in_bound:
            robot_new_node = PolyShapes.Circle(self.radius,(xPrev,yPrev))
            # check how many nodes on the tree does the new node overlaps with
            nodes_overlap_count = 0
            for k in range(shape(V)[1]-1):
                robot_old_node = PolyShapes.Circle(self.radius,(V[1,k],V[2,k]))
                if robot_new_node.overlaps(robot_old_node):
                    if  abs(thetaPrev - V_theta[k]) <   self.max_angle_overlap:
                        nodes_overlap_count += 1


            if nodes_overlap_count == 0 or (stuck > self.stuck_thres+1 and nodes_overlap_count < 2) or (stuck > self.stuck_thres+500):
                if  stuck > self.stuck_thres+1:
                    append_after_latest_node  = False

                if (stuck > self.stuck_thres+500):
                    stuck = 0
                stuck = stuck - 20
                # plotting
                if self.plotting == True:
                    self.plotPoly(path_all,'b',1)

                if self.system_print == True:
                    print "node connected"

                V = hstack((V,vstack((shape(V)[1],xPrev,yPrev))))
                V_theta = hstack((V_theta,thetaPrev))
                E = hstack((E,vstack((tree_index ,shape(V)[1]-1))))
                Other = hstack((Other,vstack((self.velocity,omega))))
                ##################### E should add omega and velocity
                connection_to_tree = True
                append_after_latest_node  = True
            else:
                append_after_latest_node = False

                if self.system_print == True:
                    print "node not connected. check goal point"

        else:
            append_after_latest_node = False

        return  V,V_theta,E,Other,stuck,append_after_latest_node, connection_to_tree


    def orientation_bound(self,theta):
        """
        make sure the returned angle is between 0 to 2*pi
        """
        while theta > 2*pi or theta < 0:
            if theta > 2*pi:
                theta = theta - 2*pi
            else:
                theta = theta + 2*pi

        return theta

    def plotMap(self,mappedRegions):
        """
        Plotting regions and obstacles with matplotlib.pyplot

        number: figure number (see on top)
        """

        #if not plt.isinteractive():
        #    plt.ion()
        #plt.hold(True)

        if self.operate_system == 1:
            for regionName,regionPoly in mappedRegions.iteritems():
                self.plotPoly(regionPoly,'k')
            plt.figure(1).canvas.draw()

    def plotPoly(self,c,string,w = 1):
        """
        Plot polygons inside the boundary
        c = polygon to be plotted with matlabplot
        string = string that specify color
        w      = width of the line plotting
        """
        if bool(c):
            for i in range(len(c)):
                #toPlot = Polygon.Polygon(c.contour(i))
                toPlot = Polygon.Polygon(c.contour(i)) & self.all
                if bool(toPlot):
                    for j in range(len(toPlot)):
                        #BoundPolyPoints = asarray(PolyUtils.pointList(toPlot.contour(j)))
                        BoundPolyPoints = asarray(PolyUtils.pointList(Polygon.Polygon(toPlot.contour(j))))
                        if self.operate_system == 2:
                            self.ax.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],string,linewidth=w)
                            self.ax.plot([BoundPolyPoints[-1,0],BoundPolyPoints[0,0]],[BoundPolyPoints[-1,1],BoundPolyPoints[0,1]],string,linewidth=w)
                        else:
                            plt.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],string,linewidth=w)
                            plt.plot([BoundPolyPoints[-1,0],BoundPolyPoints[0,0]],[BoundPolyPoints[-1,1],BoundPolyPoints[0,1]],string,linewidth=w)
                            plt.figure(1).canvas.draw()

    def data_gen(self):
        #self.ax.cla()
        for regionName,regionPoly in self.map.iteritems():
            self.plotPoly(regionPoly,'k')
        """
        #for i in range(len(self.V)):
        if shape(V)[1] <= 2:
            plt.plot(( V[1,shape(V)[1]-1],q_g[0,0]),( V[2,shape(V)[1]-1],q_g[1,0]),'b')
        else:
            plt.plot(( V[1,E[0,shape(E)[1]-1]], V[1,shape(V)[1]-1],q_g[0,0]),( V[2,E[0,shape(E)[1]-1]], V[2,shape(V)[1]-1],q_g[1,0]),'b')

        self.plotPoly(self.realRobot, 'r')
        self.plotPoly(self.robot, 'b')
        """
        pose = self.pose_handler.getPose()
        self.ax.plot(pose[0],pose[1],'bo')
        """
        self.ax.plot(self.q_g[0],self.q_g[1],'ro')
        self.plotPoly(self.overlap,'g')
        self.plotPoly(self.m_line,'b')
        """
        yield(pose[0],pose[1])
        """
        self.ax.plot(self.prev_follow[0],self.prev_follow[1],'ko')
        """

    def jplot(self):
        ani = animation.FuncAnimation(self.fig, self.scope.update, self.data_gen)
        plt.show()

class _Scope:
    def __init__(self, ax, motion, maxt=2, dt=0.02):
        self.i = 0
        self.ax = ax
        self.line, = self.ax.plot(1)
        self.ax.set_ylim(0, 1)
        self.motion = motion

    def update(self,data):
        (data1) = self.motion.data_gen()
        a = data1.next()
        self.line.set_data(a)
        self.ax.relim()
        self.ax.autoscale()
        return self.line,
