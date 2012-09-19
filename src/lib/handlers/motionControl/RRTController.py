#!/usr/bin/env python
"""
===================================================================
RRTController.py - Rapidly-Exploring Random Trees Motion Controller
===================================================================

Uses Rapidly-exploring Random Tree Algorithm to generate paths given the starting position and the goal point.
"""

import _RRTControllerHelper
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
import matplotlib.pyplot as plt
from math import sqrt, fabs , pi
import random
import matplotlib.animation as animation
import thread
import threading


class motionControlHandler:
    def __init__(self, proj, shared_data,robot_type,max_angle,plotting):
        """
        Rapidly-Exploring Random Trees alogorithm motion planning controller

        robot_type (int): Which robot is used for execution. Nao is 1, ROS is 2, ODE is 3, Pioneer is 4(default=3)
        max_angle (float): The biggest difference in angle between two nodes. The value should be within 0 to 6.28 = 2*pi. Default set to 1.047 = pi/3 (default=1.047)
        plotting (int): Enable plotting is 1 and disable plotting is 0 (default=1)
        """

        self.system_print = True
        
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
        #self.RRT_V_toPass = None
        #self.RRT_E_toPass = None
        self.point_to_go = None
        self.heading     = None
        self.E_prev      = None
        self.Velocity    = None
        self.currentRegionPoly = None
        self.nextRegionPoly    = None
        self.map               = {}
        self.all               = Polygon.Polygon()

        # Information about the robot (default set to ODE)
        if robot_type not in [1,2,3,4]:
            robot_type = 3
        self.system = robot_type

        # Information about maximum turning angle allowed
        if max_angle > 2*pi:
            max_angle = 2*pi
        if max_angle < 0:
            max_angle = 0
        self.max_angle_allowed = max_angle

        # Information about whether plotting is enabled.
        if plotting >= 1:
            self.plotting          = True
        else:
            self.plotting          = False

        # Specify the size of the robot 
        # 1: Nao ; 2: ROS; 3: 0DE; 4: Pioneer
        if  self.system == 1:
            self.radius = 0.15*1.2
        elif self.system == 2:
            self.ROSInitHandler = shared_data['ROS_INIT_HANDLER']
            print "self.ROSInitHandler.robotPhysicalWidth"+ str(self.ROSInitHandler.robotPhysicalWidth)
            self.radius = self.ROSInitHandler.robotPhysicalWidth/2
            #self.system = 1
        elif self.system == 3:
            self.radius = 5
        elif self.system == 4:
            self.radius = 0.15
            self.system = 1

        # Operate_system (int): Which operating system is used for execution.
        # Ubuntu and Mac is 1, Windows is 2
        if sys.platform in ['win32', 'cygwin']:
            self.operate_system = 2
        else:
            self.operate_system = 1            
        if self.system_print == True:
            print "operate_system is "+ str(self.operate_system)
        
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

            """
            pointArray = [x for x in self.proj.rfi.regions[current_reg].getPoints()]
            pointArray = map(self.coordmap_map2lab, pointArray)
            vertices = mat(pointArray).T
            """

            #set to zero velocity before tree is generated
            self.drive_handler.setVelocity(0, 0)
            if last:
                transFace = None
            else:
                # Determine the mid points on the faces connecting to the next region (one goal point will be picked among all the mid points)
                transFace = None
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
            
            # For plotting
            if self.operate_system == 2:
                if self.plotting == True:
                    self.ax.cla()
                else:
                    self.ax = None
            else:
                self.ax = None

            self.RRT_V,self.RRT_E,self.heading,self.E_prev = self.buildTree(\
            [pose[0], pose[1]],pose[2], self.radius,self.system,self.currentRegionPoly, self.nextRegionPoly,q_gBundle,\
            self.map,self.all,self.max_angle_allowed, self.plotting,self.operate_system)

            #self.RRT_V,self.RRT_E,self.heading,self.E_prev,self.RRT_V_toPass,self.RRT_E_toPass = self.buildTree(\
            #[pose[0], pose[1]],pose[2], self.radius,self.system,self.currentRegionPoly, self.nextRegionPoly,q_gBundle,\
            #self.map,self.all,self.max_angle_allowed, self.plotting,self.operate_system)
            """
            # map the lab coordinates back to pixels
            V_tosend = array(mat(self.RRT_V[1:,:])).T
            V_tosend = map(self.coordmap_lab2map, V_tosend)
            V_tosend = mat(V_tosend).T
            s = 'RRT:E'+"["+str(list(self.RRT_E[0]))+","+str(list(self.RRT_E[1]))+"]"+':V'+"["+str(list(self.RRT_V[0]))+","+str(list(V_tosend[0]))+","+str(list(V_tosend[1]))+"]"+':T'+"["+str(list(q_gBundle[0]))+","+str(list(q_gBundle[1]))+"]"
            #print s
            """

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        V = self.setVelocity([pose[0], pose[1]], self.RRT_V,self.RRT_E,self.heading,self.E_prev,self.radius)
        self.Velocity = V[0:2,0]
        self.heading = V[2,0]
        self.E_prev = V[3,0]
        self.previous_next_reg = next_reg

        # Pass this desired velocity on to the drive handler
        self.drive_handler.setVelocity(V[0,0], V[1,0], pose[2])
        RobotPoly = Polygon.Shapes.Circle(self.radius,(pose[0],pose[1]))
        
        # check if robot is inside the current region
        departed = not self.currentRegionPoly.covers(RobotPoly)
        arrived  = self.nextRegionPoly.covers(RobotPoly)

        if departed and (not arrived) and (time.time()-self.last_warning) > 0.5:
            # Figure out what region we think we stumbled into
            for r in self.proj.rfi.regions:
                pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                vertices = mat(pointArray).T

                if is_inside([pose[0], pose[1]], vertices):
                    #print "I think I'm in " + r.name
                    #print pose
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

    def setVelocity(self,p, V, E, heading,E_prev,radius, last=False):
        """
        This function calculates the velocity for the robot with RRT.
        The inputs are (given in order):
            p        = the current x-y position of the robot
            E        = edges of the tree  (2 x No. of nodes on the tree)
            V        = points of the tree (2 x No. of vertices)
            heading  = index of the previous heading point on the tree
            E_prev   = current index of E
            last = True, if the current region is the last region
                 = False, if the current region is NOT the last region
        """

        pose     = mat(p).T
        #dis_cur = distance between current position and the next point
        dis_cur  = vstack((V[1,E[1,E_prev]],V[2,E[1,E_prev]]))- pose
        if norm(dis_cur) < 1.5*radius:         # go to next point
            if not heading == shape(V)[1]-1:
                E_prev = E_prev + 1
                dis_cur  = vstack((V[1,E[1,E_prev]],V[2,E[1,E_prev]]))- pose
                heading = E[1,E_prev]
            else:
                dis_cur  = vstack((V[1,E[1,E_prev]],V[2,E[1,E_prev]]))- vstack((V[1,E[0,E_prev]],V[2,E[0,E_prev]]))
        Vel = zeros([4,1])


        Vel[0:2,0] = dis_cur/norm(dis_cur)*0.5                    #TUNE THE SPEED LATER
        Vel[2,0]   = heading
        Vel[3,0]   = E_prev

        return Vel

    def buildTree(self,p,theta, R, system, regionPoly,nextRegionPoly,q_gBundle,\
    mappedRegions,allRegions,max_angle_allowed, plotting, operate_system, last=False):

        """
        This function builds the RRT tree.
        
        p                : x,y position of the robot
        theta            : current orientation of the robot
        R                : radius of the robot
        system           : determine step_size and radius for the example
        regionPoly       : current region polygon
        nextRegionPoly   : next region polygon
        q_gBundle        : coordinates of q_goals that the robot can reach
        mappedRegions    : region polygons
        allRegions       : polygon that includes all the region
        max_angle_allowed: the difference in angle between the two nodes allowed (between 0 to 2*pi)
        plotting         : True if plotting is enabled, False- disabled
        operate_system   : Which operating system is used for execution. Ubuntu and Mac is 1, Windows is 2
        """
        finish_print       = 0       #set to 1 to print finish E and V before trimming
        max_angle = max_angle_allowed


        ## 1: Nao and Pioneer; 2: ROS ; 3: ODE
        #  Time step: for calculation of x, y position
        #  Step_size: the length of each step in control space
        #  Velocity : Velocity of the robot in m/s in control space
        
        
        if system == 1:    ## Nao
            step_size  = 0.2      #set the step_size for points be 1/5 of the norm  ORIGINAL = 0.4
            timeStep = 5
            velocity  = 0.05 
        elif system == 2:
            step_size  = 0.2
            timeStep = 8
            velocity  = 0.08
        elif system == 3:
            step_size  = 15
            timeStep = 10
            velocity = 2    # 1.5
        #############tune velocity OMEGA, TIME STEP
    

        BoundPoly       = regionPoly       # Boundary polygon = current region polygon
        radius          = R
        q_init          = mat(p).T
        V               = vstack((0,q_init))
        V_theta         = array([theta])
        
        if operate_system == 1 and plotting == True:
            plt.cla()
            self.plotMap(mappedRegions) 
            plt.plot(p[0],p[1],'ko')           
            
        #!!! CONTROL SPACE: generate a list of omega for random sampling
        omegaLowerBound = -math.pi/20      # upper bound for the value of omega
        omegaUpperBound = math.pi/20       # lower bound for the value of omega
        omegaStepSize   = 20
        omega_range = linspace(omegaLowerBound,omegaUpperBound,omegaStepSize)
        omega_range_abso = linspace(omegaLowerBound*4,omegaUpperBound*4,omegaStepSize*4)    # range used when stuck > stuck_thres
        edgeX    = []
        edgeY    = []

        # check faces of the current region for goal points
        E = [[],[]]
        Other = [[],[]]
        path     = 0          # if path formed then = 1
        stuck    = 0          # count for changing the range of sampling omega
        stuck_thres = 20     # threshold for changing the range of sampling omega

        if self.system_print == True:
            print "plotting in buildTree is " + str(plotting)
        if plotting == True:
            if not plt.isinteractive():
                plt.ion()
            plt.hold(True)

        while path == 0:
            #step -1: try connection to q_goal (generate path to goal)
            i = 0
            """
            if self.system_print == True:
                print "step -1 "
            """
            
            # pushing possible q_goals into the current region (ensure path is covered by the current region polygon)
            q_pass = [[],[],[]]
            q_pass_dist = []
            q_gBundle = mat(q_gBundle)
            while i < q_gBundle.shape[1]: 
                q_g = q_gBundle[:,i]+(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])*1.5*radius    ##original 2*radius
                trial = 1
                if not BoundPoly.isInside(q_g[0],q_g[1]):
                    trial = 2
                    q_g = q_gBundle[:,i]-(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])*1.5*radius    ##original 2*radius

                #forming polygon for path checking
                """
                cross_goal     = cross(vstack((q_g-vstack((V[1,shape(V)[1]-1],V[2,shape(V)[1]-1])),0)).T,hstack((0,0,1)))
                cross_goal       = cross_goal.T
                move_vector_goal = radius*cross_goal[0:2]/sqrt((cross_goal[0,0]**2 + cross_goal[1,0]**2))
                upperEdgeG   = hstack((vstack((V[1,shape(V)[1]-1],V[2,shape(V)[1]-1])),q_g)) + hstack((move_vector_goal,move_vector_goal))
                lowerEdgeG   = hstack((vstack((V[1,shape(V)[1]-1],V[2,shape(V)[1]-1])),q_g)) - hstack((move_vector_goal,move_vector_goal))
                EdgePolyGoal    = Polygon.Polygon((tuple(array(lowerEdgeG[:,0].T)[0]),tuple(array(upperEdgeG[:,0].T)[0]),tuple(array(upperEdgeG[:,1].T)[0]),tuple(array(lowerEdgeG[:,1].T)[0])))
                """
                
                EdgePolyGoal = PolyShapes.Circle(radius,(q_g[0,0],q_g[1,0])) + PolyShapes.Circle(radius,(V[1,shape(V)[1]-1],V[2:,shape(V)[1]-1]))
                EdgePolyGoal = PolyUtils.convexHull(EdgePolyGoal)
                dist = norm(q_g - V[1:,shape(V)[1]-1])
                
                #check connection to goal
                connect_goal = BoundPoly.covers(EdgePolyGoal)   #check coverage of path from new point to goal
                
                # compare orientation difference
                thetaPrev = V_theta[shape(V)[1]-1]
                theta_orientation = abs(arctan((q_g[1,0]- V[2,shape(V)[1]-1])/(q_g[0,0]- V[1,shape(V)[1]-1])))
                if thetaPrev < 0:
                    if q_g[1,0] > V[2,shape(V)[1]-1]:
                        if q_g[0,0] < V[1,shape(V)[1]-1]: # second quadrant
                            theta_orientation = -2*pi + theta_orientation
                        elif q_g[0,0] > V[1,shape(V)[1]-1]: # first quadrant
                            theta_orientation = -pi -theta_orientation
                    elif q_g[1,0] < V[2,shape(V)[1]-1]:
                        if q_g[0,0] < V[1,shape(V)[1]-1]: #third quadrant
                            theta_orientation = -pi + theta_orientation
                        elif q_g[0,0] > V[1,shape(V)[1]-1]: # foruth quadrant
                            theta_orientation =  - theta_orientation
                else:
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

                ################################## PRINT PLT #################
                if connect_goal :
                    if plotting == True:
                        if operate_system == 1:
                            plt.suptitle('Rapidly-exploring Random Tree', fontsize=12)
                            plt.xlabel('x')
                            plt.ylabel('y')
                            if shape(V)[1] <= 2:
                                plt.plot(( V[1,shape(V)[1]-1],q_g[0,0]),( V[2,shape(V)[1]-1],q_g[1,0]),'b')
                            else:
                                plt.plot(( V[1,E[0,shape(E)[1]-1]], V[1,shape(V)[1]-1],q_g[0,0]),( V[2,E[0,shape(E)[1]-1]], V[2,shape(V)[1]-1],q_g[1,0]),'b')
                            plt.figure(1).canvas.draw()
                        else:
                            BoundPolyPoints = asarray(PolyUtils.pointList(BoundPoly))
                            self.ax.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
                            if shape(V)[1] <= 2:
                                self.ax.plot(( V[1,shape(V)[1]-1],q_g[0,0]),( V[2,shape(V)[1]-1],q_g[1,0]),'b')
                            else:
                                self.ax.plot(( V[1,E[0,shape(E)[1]-1]], V[1,shape(V)[1]-1],q_g[0,0]),( V[2,E[0,shape(E)[1]-1]], V[2,shape(V)[1]-1],q_g[1,0]),'b')

                # if connection to goal can be established and the max change in orientation of the robot is smaller than max_angle, tree is said to be completed.

                if connect_goal and abs(theta_orientation - thetaPrev) < max_angle:
                    path = 1
                    q_pass = hstack((q_pass,vstack((i,q_g))))
                    q_pass_dist = hstack((q_pass_dist,dist))

                i = i + 1
            
            
            if self.system_print == True:
                print "checked goal points"
                if shape(V)[1]-1 > node_thres-10: 
                    print "may erase the current tree and generate a new one. count:" + str(shape(V)[1]-1 - node_thres)
            
            self.E = E
            self.V = V
            # connection to goal has established
            # Obtain the closest goal point that path can be formed.
            if path == 1:
                if shape(q_pass_dist)[0] == 1:
                    cols = 0
                else:
                    (cols,) = nonzero(q_pass_dist == min(q_pass_dist))
                    cols = asarray(cols)[0]
                q_g = q_pass[1:,cols]   ###Catherine
                q_g = q_g-(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])*3*radius   #org 3
                if not nextRegionPoly.isInside(q_g[0],q_g[1]):
                    q_g = q_g+(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])*6*radius   #org 3

                if plotting == True :
                    if operate_system == 1:
                        plt.plot(q_g[0,0],q_g[1,0],'ko')
                        plt.figure(1).canvas.draw()
                    else:
                        self.ax.plot(q_g[0,0],q_g[1,0],'ko')

                # trim the path connecting current node to goal point into pieces if the path is too long now
                numOfPoint = floor(norm(V[1:,shape(V)[1]-1]- q_g)/step_size)
                if numOfPoint < 3:
                    numOfPoint = 3
                x = linspace( V[1,shape(V)[1]-1], q_g[0,0], numOfPoint )
                y = linspace( V[2,shape(V)[1]-1], q_g[1,0], numOfPoint )
                for i in range(x.shape[0]):
                    if i != 0:
                        V = hstack((V,vstack((shape(V)[1],x[i],y[i]))))
                        E = hstack((E,vstack((shape(V)[1]-2,shape(V)[1]-1))))

            # path is not formed, try to append points onto the tree
            if path == 0:
                success     = 0           # whether adding a new point is successful
                hit         = 0           # whether adding more new points are successful 1 = not successful should quit
                Icurrent    = []          # to keep track of the index of the closest point to q_n

                while success == 0 and hit == 0:
                    #and hit_count <= 2: 
                    """
                    if self.system_print == True:
                        print "In control space generating path,stuck = " + str(stuck)
                    """                  
                    if stuck > stuck_thres:
                        # increase the range of omega since path cannot ge generated
                        omega = random.choice(omega_range_abso)
                    else:
                        #!!!! CONTROL SPACE STEP 1 - generate random omega
                        omega = random.choice(omega_range)


                    #!!!! CONTROL SPACE STEP 2 - pick a random point on the tree
                    if success == 1:
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
                    path_robot = PolyShapes.Circle(radius,(xPrev,yPrev))
                    while j <= timeStep:
                        xOrg      = xPrev
                        yOrg      = yPrev
                        xPrev     = xPrev + velocity/omega*(sin(omega* 1 + thetaPrev)-sin(thetaPrev))
                        yPrev     = yPrev - velocity/omega*(cos(omega* 1 + thetaPrev)-cos(thetaPrev))
                        thetaPrev = omega* 1 + thetaPrev
                        path_robot = path_robot + PolyShapes.Circle(radius,(xPrev,yPrev))

                        j = j + 1

                    path_all = PolyUtils.convexHull(path_robot)
                    in_bound = BoundPoly.covers(path_all)
                    """
                    # plotting
                    if plotting == True:
                        self.plotPoly(path_all,'r',1) 
                    """                     

                    stuck = stuck + 1
                    
                    if in_bound: 
                        #print V
                        robot_new_node = PolyShapes.Circle(radius,(xPrev,yPrev))
                        second_success_check = 1 
                        second_success_check_count = 0
                        for k in range(shape(V)[1]-1):
                            robot_old_node = PolyShapes.Circle(radius,(V[1,k],V[2,k])) 
                            if robot_new_node.overlaps(robot_old_node):
                                second_success_check = 0
                                second_success_check_count += 1
                        
                        """                        
                        x = []
                        y = []
                        for k in  PolyUtils.pointList(path_all):
                            x = hstack((x,k[0]))
                            y = hstack((y,k[1]))
                        """
                        
                        if second_success_check == 1 or (stuck > stuck_thres+1 and second_success_check_count < 2) or success == 1 or (stuck > stuck_thres+100):  
                            if  stuck > stuck_thres+1:
                                hit  = 1  
                            stuck = stuck - 20              
                            # plotting
                            if plotting == True:
                                self.plotPoly(path_all,'b',1) 
                            """    
                            if self.system_print == True:
                                print "node connected" 
                            """      
                            V = hstack((V,vstack((shape(V)[1],xPrev,yPrev))))
                            V_theta = hstack((V_theta,thetaPrev))
                            E = hstack((E,vstack((tree_index ,shape(V)[1]-1))))
                            Other = hstack((Other,vstack((velocity,omega))))
                            ##################### E should add omega and velocity
                            success = 1
                        else:
                            if success == 1:
                                hit = 1
                            """
                            if self.system_print == True:
                                print "node not connected. check goal point"
                            """ 
                    else:
                        if success == 1:
                            hit = 1
                    
                    

        if finish_print == 1:
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
        """
        # generate V to be sent to SimGUI to plot
        V_toPass = V[0:,0]
        E_toPass = [[],[]]
        for i in range(shape(E)[1]):
            V_toPass = hstack((V_toPass,vstack((i,V[1,E[1,i-1]],V[2,E[1,i-1]]))))
            E_toPass = hstack((E_toPass,vstack((i-1,i))))
        """

        ####print with matlib
        if plotting ==True :
            if operate_system == 1:
                plt.plot(V[1,:],V[2,:],'b')
                for i in range(shape(E)[1]-1):
                    plt.text(V[1,E[0,i]],V[2,E[0,i]], V[0,E[0,i]], fontsize=12)
                    plt.text(V[1,E[1,i]],V[2,E[1,i]], V[0,E[1,i]], fontsize=12)
                plt.figure(1).canvas.draw()
            else:
                BoundPolyPoints = asarray(PolyUtils.pointList(BoundPoly))
                self.ax.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
                self.ax.plot(V[1,:],V[2,:],'b')
                for i in range(shape(E)[1]-1):
                    self.ax.text(V[1,E[0,i]],V[2,E[0,i]], V[0,E[0,i]], fontsize=12)
                    self.ax.text(V[1,E[1,i]],V[2,E[1,i]], V[0,E[1,i]], fontsize=12)

        heading  = E[0,0]
        # parse string for RRT printing in GUI (in format: RRT:E[[1,2,3]]:V[[1,2,3]])
        V = array(V)
        #V_toPass = array(V_toPass)
        #E_toPass = array(E_toPass)
        #return V, E, heading,0,V_toPass,E_toPass
        return V, E, heading,0

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
