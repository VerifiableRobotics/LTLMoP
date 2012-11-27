#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
######################################################################
"""
===================================================================
OMPLController.py - Open Motion Planning Library Motion Controller
===================================================================

Uses Open Motion Planning Library developed by Rice University to generate paths.
"""

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import control as oc
    from ompl import geometric as og
except:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import basename, abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))),'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import control as oc
    from ompl import geometric as og
from functools import partial
from time import sleep
from math import fabs
from numpy import *
from __is_inside import *
import math
import sys,os, time
from scipy.linalg import norm
from numpy.matlib import zeros
import scipy as Sci
import scipy.linalg
import Polygon, Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
import Tkinter as Tk
from mpl_toolkits.mplot3d import Axes3D
from math import sqrt, fabs , pi
import random
import thread
import threading


class motionControlHandler:
    def __init__(self, proj, shared_data,Space_Dimension,planner,robot_type,Geometric_Control,plotting):
        """
        Space_Dimension(int): dimension of the space operating in. Enter 2 for 2D and 3 for 3D. Only quadrotor in ROS is supported for 3D now.(default=2)
        planner(string): Planner to be used. Enter RRT,KPIECE1 or PRM, RRTConnect. (default='PRM')
        robot_type (int): Which robot is used for execution. Nao is 1, ROS is 2, ODE is 3, Pioneer is 4(default=3)
        Geometric_Control(string): Specify if you want to planner to sample in geometric or control space. G for geometric and C for control. (default='G')
        plotting (bool): Check the box to enable plotting (default=True)
        """
        """
        maxHeight(float): maximum height for the 3D space. Units:m(default=1)
        max_angle_goal (float): The biggest difference in angle between the new node and the goal point that is acceptable. If it is bigger than the max_angle, the new node will not be connected to the goal point. The value should be within 0 to 6.28 = 2*pi. Default set to 6.28 = 2*pi (default=6.28)
        max_angle_overlap (float): difference in angle allowed for two nodes overlapping each other. If you don't want any node overlapping with each other, put in 2*pi = 6.28. Default set to 1.57 = pi/2 (default=1.57)
        """
        
        #Parameters
        self.system_print = True
        self.currentRegionPoly  = None                  # polygon of the current region
        self.nextRegionPoly     = None                  # polygon of the next region
        self.map                = {}                    # dictionary of polygons of different regions
        self.all                = Polygon.Polygon()     # polygon of the boundary
        self.OMPLpath           = None
        self.trans_matrix       = mat([[0,1],[-1,0]])   # transformation matrix for find the normal to the vector 
        
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.h_instance['drive']
        self.pose_handler = proj.h_instance['pose']
    
        # Get information about regions
        self.proj              = proj
        self.coordmap_map2lab  = proj.coordmap_map2lab
        self.coordmap_lab2map  = proj.coordmap_lab2map
        self.last_warning      = 0
        self.previous_next_reg = None
        
        # Information about the spoce dimension
        if Space_Dimension not in [2,3]:
            Space_Dimension = 2
        self.Space_Dimension = Space_Dimension
        
        # Information about the planner
        if planner not in ['RRT','KPIECE1','PRM','RRTConnect','EST']:
            planner = 'PRM'
        self.planner = planner
        
        # Information about the geometric or control space
        if Geometric_Control not in ['G','C']:
            Geometric_Control = 'G'
        self.Geometric_Control = Geometric_Control
        
        # Information about the robot (default set to ODE)
        if robot_type not in [1,2,3,4]:
            robot_type = 3
        self.system = robot_type
        
        # Information about whether plotting is enabled.
        if plotting is True:
            self.plotting          = True
        else:
            self.plotting          = False            
            
        # Operate_system (int): Which operating system is used for execution.
        # Ubuntu and Mac is 1, Windows is 2
        if sys.platform in ['win32', 'cygwin']:
            self.operate_system = 2
        else:
            self.operate_system = 1   
                     
        if self.system_print == True:
            print "Operate_system: "+ str(self.operate_system)
            print "Planner: " + str(self.planner)
            print "Geometric/Control space: " + str(self.Geometric_Control)
            print "Space Dimension: " + str(self.Space_Dimension)
        
        # Generate polygon for regions in the map        
        
        # getting raw region inputs from user. To be used for 3D path planning
        self.original_regions = self.proj.loadRegionFile()
        #print self.original_regions.filename
        print "MAXHEIGHT:" +str(self.original_regions.getMaximumHeight())
        #print self.proj.rfi.filename
        
        # Information about the maximum height in z direction
        self.maxHeight = self.original_regions.getMaximumHeight()
           
        self.map = {'polygon':{},'original_name':{},'height':{}}
        for region in self.proj.rfi.regions:
            self.map['polygon'][region.name] = self.createRegionPolygon(region)
            for n in range(len(region.holeList)): # no of holes
                self.map['polygon'][region.name] -= self.createRegionPolygon(region,n)
        
        # map the names back to the old original names specified by the user  
        for rname, rlist in self.proj.regionMapping.iteritems():
            #print rname,rlist[0]  
            self.map['original_name'][rlist[0]] = rname
            #store the height of the regions
            for region in self.original_regions.regions:
                if region.name.lower() == rname.lower():
                    self.map['height'][rlist[0]]  = region.height
                    
        # ****** only substract.. never add the region.. so obstacle in the middle will be taken care of   
        
        self.original_map = {'polygon':{},'height':{},'isObstacle':{}}   
        for region in self.original_regions.regions:
            self.original_map['polygon'][region.name] = self.createRegionPolygon(region)
            self.original_map['height'][region.name]  = region.height
            self.original_map['isObstacle'][region.name] = region.isObstacle
        """   
        if self.system_print is True:
            print self.map
            for region in self.original_regions.regions:
                print "name: "+ str(region.name) + " isObstacle:" +str(region.isObstacle) + " height:" +str(region.height)
            print self.original_map
        """
        
        # building the planner dictionary
        self.planner_dictionary = {'G':{},'C':{}}
        self.planner_dictionary['G']['PRM'] = og.PRM
        self.planner_dictionary['G']['RRT'] = og.RRT
        self.planner_dictionary['G']['KPIECE1'] = og.KPIECE1
        self.planner_dictionary['G']['RRTConnect'] = og.RRTConnect
        self.planner_dictionary['G']['EST'] = og.EST
        #self.planner_dictionary['C']['PRM'] = oc.PRM
        self.planner_dictionary['C']['RRT'] = oc.RRT
        self.planner_dictionary['C']['KPIECE1'] = oc.KPIECE1
        
        # Generate the boundary polygon 
        for regionName,regionPoly in self.map['polygon'].iteritems():
            self.all += regionPoly
        
        # Specify the size of the robot 
        # 1: Nao ; 2: ROS; 3: 0DE; 4: Pioneer
        #  self.radius: radius of the robot (m)
        if  self.system == 1:
            self.radius = 0.15*1.2
        elif self.system == 2:
            self.ROSInitHandler = shared_data['ROS_INIT_HANDLER']
            self.radius = self.ROSInitHandler.robotPhysicalWidth/2
            if self.ROSInitHandler.modelName == 'quadrotor':
                self.height = 0.30 #(m) height of the robot
        elif self.system == 3:
            self.radius = 5
            self.height = 0.30     ########### CHANGE TO BE DELETED
        elif self.system == 4:
            self.radius = 0.15
        

        if self.plotting == True: 
            app = _MyTkApp()
            app.start()            
            self.TkApp = app.getSharedData()
            self.fig = self.TkApp.fig
            self.ax  = self.TkApp.ax
            self.ax.legend()            
            self.BoundaryMaxMin = self.all.boundingBox()  #0-3:xmin, xmax, ymin and ymax          
            self.plotMap()     
            self.setPlotLimitXYZ()

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.
        Returns ``True`` if we've reached the destination region.
        """
        # Find our current configuration
        pose = self.pose_handler.getPose()
        
        if self.plotting == True:
            
            if self.operate_system == 1:
                if self.Space_Dimension == 3:
                    self.ax.plot([pose[0]],[pose[1]],[pose[3]],'ko')  
                else:
                    self.ax.plot([pose[0]],[pose[1]],'ko') 
                self.setPlotLimitXYZ()
                self.ax.get_figure().canvas.draw()           
                    
        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return True

        # Check if Vicon has cut out
        # TODO: this should probably go in posehandler?
        if math.isnan(pose[2]):
            print "WARNING: No Vicon data! Pausing."
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            time.sleep(1)
            return False

        ###This part will be run when the robot goes to a new region, otherwise, the original tree will be used.
        if not self.previous_next_reg == next_reg:
            
            # plotting current pose and the map
            if self.operate_system == 1 and self.plotting == True:
                self.ax.cla()
                self.plotMap()
                if self.Space_Dimension == 3:
                    self.ax.plot([pose[0]],[pose[1]],[pose[3]],'ko')  
                else:
                    self.ax.plot([pose[0]],[pose[1]],'ko')             

            # Entered a new region. New tree should be formed.
            if self.Space_Dimension == 3:
                self.nextRegionPoly    = self.original_map['polygon'][self.map['original_name'][self.proj.rfi.regions[next_reg].name]]
                self.currentRegionPoly = self.original_map['polygon'][self.map['original_name'][self.proj.rfi.regions[current_reg].name]]
                """
                ############### CAN BE DELETED LATER)
                self.plotPoly(self.nextRegionPoly,'r')
                print "next region:" +str(self.nextRegionPoly)
                self.plotPoly(self.currentRegionPoly,'b')
                plt.show()
                """
                self.nextAndcurrentRegionPoly = self.nextRegionPoly+self.currentRegionPoly
            else:
                self.nextRegionPoly    = self.map['polygon'][self.proj.rfi.regions[next_reg].name]
                self.currentRegionPoly = self.map['polygon'][self.proj.rfi.regions[current_reg].name]
                self.nextAndcurrentRegionPoly = self.nextRegionPoly+self.currentRegionPoly
            
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
                goalPoints   = [[],[]] # list of goal points (midpoints of transition faces)
                face_normal = [[],[]] # normal of the trnasition faces
                for i in range(len(self.proj.rfi.transitions[current_reg][next_reg])):
                    pointArray_transface = [x for x in self.proj.rfi.transitions[current_reg][next_reg][i]]
                    transFace = asarray(map(self.coordmap_map2lab,pointArray_transface))
                    coord_x = (transFace[0,0] +transFace[1,0])/2    #mid-point coordinate x
                    coord_y = (transFace[0,1] +transFace[1,1])/2    #mid-point coordinate y
                    goalPoints = hstack((goalPoints,vstack((coord_x,coord_y))))
                
                    #find the normal vector to the face
                    face          = transFace[0,:] - transFace[1,:]
                    distance_face = norm(face)
                    normal        = face/distance_face * self.trans_matrix
                    face_normal   = hstack((face_normal,vstack((normal[0,0],normal[0,1]))))
                
                # move the goal points to the next region
                q_gBundle = mat(goalPoints)
                face_normal = mat(face_normal)    
                for i in range(q_gBundle.shape[1]):           
                    q_g = q_gBundle[:,i]+face_normal[:,i]*1.5*self.radius    ##original 2*self.radius
                    if not self.nextRegionPoly.isInside(q_g[0],q_g[1]):
                        q_g = q_gBundle[:,i]-face_normal[:,i]*1.5*self.radius    ##original 2*self.radius
                    goalPoints[0,i] = q_g[0,0]
                    goalPoints[1,i] = q_g[1,0]
                    
                
                if transFace is None:
                    print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.proj.rfi.regions[current_reg].name, self.proj.rfi.regions[next_reg].name)
                    
            self.OMPLpath = self.plan(goalPoints,self.proj.rfi.regions[current_reg].name,self.proj.rfi.regions[next_reg].name,0) 
            self.currentState = 1
            print "Going to generate velocity"
            #print OMPLpath
                
            
        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        if self.Space_Dimension == 3:
            self.Velocity = self.getVelocity([pose[0],pose[1],pose[3]], self.OMPLpath)
        else:
            self.Velocity = self.getVelocity([pose[0], pose[1]], self.OMPLpath)
        self.previous_next_reg = next_reg
        
        """
        # FOR ROBERT
        self.Node = self.getNode([pose[0], pose[1]], self.OMPLpath)
        print "self.Node:" + str(self.Node) 
        self.drive_handler.setDestination(self.Node[0,0], self.Node[1,0], pose[2])
        """

        # Pass this desired velocity on to the drive handler
        if self.Space_Dimension == 3:
            self.drive_handler.setVelocity(self.Velocity[0,0], self.Velocity[1,0],pose[2],self.Velocity[2,0])
        else:
            self.drive_handler.setVelocity(self.Velocity[0,0], self.Velocity[1,0],pose[2])
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
            
    def getVelocity(self,p, OMPLpath, last=False):
        """
        This function calculates the velocity for the robot with RRT.
        The inputs are (given in order):
            p        = the current x-y position of the robot
            OMPLpath = path information of the planner
            last = True, if the current region is the last region
                 = False, if the current region is NOT the last region
        """

        pose     = mat(p).T
        """
        if self.system_print == True:
            print (OMPLpath.getSolutionPath().getState(self.currentState))[0]   # x-coordinate of the current state  
        """ 
        if self.Space_Dimension == 3:    
            #print (OMPLpath.getSolutionPath().getState(self.currentState))[0]
            
            #dis_cur = distance between current position and the next point
            #dis_cur  = vstack(((OMPLpath.getSolutionPath().getState(self.currentState))[0],(OMPLpath.getSolutionPath().getState(self.currentState))[1]))- pose
            dis_cur  = vstack(((OMPLpath.getSolutionPath().getState(self.currentState)).getX(),(OMPLpath.getSolutionPath().getState(self.currentState)).getY(),(OMPLpath.getSolutionPath().getState(self.currentState)).getZ()))- pose

            if norm(dis_cur) < 1.5*self.radius:         # go to next point
                #print self.currentState == OMPLpath.getSolutionPath().getStateCount()
                #print "self.currentState: " + str(self.currentState)
                #print "OMPLpath.getSolutionPath().getStateCount(): " + str(OMPLpath.getSolutionPath().getStateCount())
                if not (self.currentState+1) == OMPLpath.getSolutionPath().getStateCount():
                    #print "adding current state number"
                    self.currentState = self.currentState + 1
                    #dis_cur  = vstack(((OMPLpath.getSolutionPath().getState(self.currentState))[0],(OMPLpath.getSolutionPath().getState(self.currentState))[1]))- pose
                    dis_cur  = vstack(((OMPLpath.getSolutionPath().getState(self.currentState)).getX(),(OMPLpath.getSolutionPath().getState(self.currentState)).getY(),(OMPLpath.getSolutionPath().getState(self.currentState)).getZ()))- pose
                    #print "get new distance"
            
            Vel = zeros([3,1])
            Vel[0:3,0] = dis_cur/norm(dis_cur)*0.5                    #TUNE THE SPEED LATER
            
        else:
            dis_cur  = vstack(((OMPLpath.getSolutionPath().getState(self.currentState)).getX(),(OMPLpath.getSolutionPath().getState(self.currentState)).getY()))- pose[0:2]

            if norm(dis_cur) < 1.5*self.radius:         # go to next point
                if not (self.currentState+1) == OMPLpath.getSolutionPath().getStateCount():
                    # head to the next node
                    self.currentState = self.currentState + 1
                    dis_cur  = vstack(((OMPLpath.getSolutionPath().getState(self.currentState)).getX(),(OMPLpath.getSolutionPath().getState(self.currentState)).getY()))- pose[0:2]
            Vel = zeros([2,1])
            Vel[0:2,0] = dis_cur/norm(dis_cur)*0.5                    #TUNE THE SPEED LATER
        return Vel
    
    def getNode(self,p, OMPLpath, last=False):
        """

        This function return the heading node of the robot. (for 2D only now)
        The inputs are (given in order):
            p        = the current x-y position of the robot

            E        = edges of the tree  (2 x No. of nodes on the tree)
            V        = points of the tree (2 x No. of vertices)
            last = True, if the current region is the last region
                 = False, if the current region is NOT the last region

        """

        pose     = mat(p).T
        
        #dis_cur = distance between current position and the next point
        dis_cur  = vstack(((OMPLpath.getSolutionPath().getState(self.currentState)).getX(),(OMPLpath.getSolutionPath().getState(self.currentState)).getY()))- pose[0:2]
        
        if norm(dis_cur) < 1.5*self.radius:         # go to next point
            if not (self.currentState+1) == OMPLpath.getSolutionPath().getStateCount():
                # head to the next node
                self.currentState = self.currentState + 1
        
        Node = zeros([2,1])
        Node[0,0] = (OMPLpath.getSolutionPath().getState(self.currentState)).getX()
        Node[1,0] = (OMPLpath.getSolutionPath().getState(self.currentState)).getY()
        return Node 
                         
    
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
                
    def plotMap(self):
        """
        Plotting regions and obstacles with matplotlib.pyplot

        number: figure number (see on top)
        """

        if self.operate_system == 1:
            for regionName,regionPoly in self.map['polygon'].iteritems():
                self.plotPoly(regionPoly,'k')
    
    def setPlotLimitXYZ(self):
        """
        Set the limits for the plot on x, y and z axis
        """
        self.ax.set_xlim3d(self.BoundaryMaxMin[0], self.BoundaryMaxMin[1])
        self.ax.set_ylim3d(self.BoundaryMaxMin[2], self.BoundaryMaxMin[3])
        self.ax.set_zlim3d(-0.05,self.maxHeight)

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
                            self.ax.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],string,linewidth=w)
                            self.ax.plot([BoundPolyPoints[-1,0],BoundPolyPoints[0,0]],[BoundPolyPoints[-1,1],BoundPolyPoints[0,1]],string,linewidth=w)
                            self.setPlotLimitXYZ()
                            #self.ax.get_figure().canvas.draw()
                            
    # return an obstacle-based sampler
    def allocOBValidStateSampler(self,si):
        # we can perform any additional setup / configuration of a sampler here,
        # but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
        return ob.ObstacleBasedValidStateSampler(si)

    # return an instance of my sampler
    def alloc_MyValidStateSampler(self,si):
        return _MyValidStateSampler(si)

    # This function is needed, even when we can write a sampler like the one
    # above, because we need to check path segments for validity
    def isStateValid(self,state):
        # Let's pretend that the validity check is computationally relatively
        # expensive to emphasize the benefit of explicitly generating valid
        # samples
        sleep(.001)
        # Valid states satisfy the following constraints:
        # inside the current region and the next region   
        #print "state: " + str(state)
        #print "state.getX(): " + str(state.getX())
        if self.Space_Dimension == 3:
            """
            # check if the robot is in the region, below the maxHeight and above 0
            #print "xy:"+ str(self.nextAndcurrentRegionPoly.covers(PolyShapes.Circle(self.radius,(state.getX(),state.getY())))) + " top:" + str((state.getZ()+self.height) < self.maxHeight) + " bottom: " + str((state.getZ()-self.height) > 0 )
            #self.ax.plot([state.getX()],[state.getY()],[state.getZ()],'ro')
            #self.setPlotLimitXYZ()
            #self.ax.get_figure().canvas.draw()
            return self.nextAndcurrentRegionPoly.covers(PolyShapes.Circle(self.radius,(state.getX(),state.getY()))) and (state.getZ()+self.height/2) < self.maxHeight and (state.getZ()-self.height/2) > 0
            """
            region_considered = Polygon.Polygon(self.nextAndcurrentRegionPoly)
            bottom = state.getZ()-self.height/2  # bottom of the robot
            for i, name in enumerate(self.original_map['polygon']):
                if self.original_map['isObstacle'][name] is True:
                    if self.original_map['height'][name] >= bottom:
                        region_considered -=self.original_map['polygon'][name]
            
            return region_considered.covers(PolyShapes.Circle(self.radius,(state.getX(),state.getY()))) and (state.getZ()+self.height/2) < self.maxHeight and (state.getZ()-self.height/2) > 0
        else: 
            return self.nextAndcurrentRegionPoly.covers(PolyShapes.Circle(self.radius,(state.getX(),state.getY())))
             
    
    # This function generates control space needed information
    def propagate(self, start, control, duration, state):
        # control[0] = velocity
        # control[1] = omega
        # duration  = setPropagationStepSize (in plan)
        print >>sys.__stdout__,"control[0]: " + str(control[0])
        print >>sys.__stdout__,"control[1]: " + str(control[1])
        print >>sys.__stdout__,"duration: " + str(duration)
        state.setX( start.getX() + control[0] * duration * cos(start.getYaw()) )
        state.setY( start.getY() + control[0] * duration * sin(start.getYaw()) )
        state.setYaw(start.getYaw() + control[1] * duration)

    def plan(self,goalPoints,current_region,next_region,samplerIndex):
        """
        goal points: array that contains the coordinates of all the possible goal states
        current_reg: name of the current region (p1 etc)
        next_reg   : name of the next region (p1 etc)
        """
        # construct the state space we are planning in
        #space = ob.RealVectorStateSpace(self.Space_Dimension)
        if self.Space_Dimension == 2:
            space = ob.SE2StateSpace()
        else:
            space = ob.SE3StateSpace()

        # set the bounds
        bounds = ob.RealVectorBounds(self.Space_Dimension)    
        BoundaryMaxMin = self.all.boundingBox()  #0-3:xmin, xmax, ymin and ymax
        bounds.setLow(0,BoundaryMaxMin[0])    # 0 stands for x axis
        bounds.setHigh(0,BoundaryMaxMin[1])
        bounds.setLow(1,BoundaryMaxMin[2])    # 1 stands for y axis
        bounds.setHigh(1,BoundaryMaxMin[3])
        
        if self.Space_Dimension == 3:
            bounds.setLow(2,0)  # 2 stands for z axis
            bounds.setHigh(2,self.maxHeight)
        
        space.setBounds(bounds)
        if self.system_print == True:
            print "The bounding box of the boundary is: " + str(self.all.boundingBox() )
            print "The volume of the bounding box is: " + str(bounds.getVolume())
        
        if self.Geometric_Control == 'C':
            # create a control space
            cspace = oc.RealVectorControlSpace(space, self.Space_Dimension)

            # set the bounds for the control space
            cbounds = ob.RealVectorBounds(self.Space_Dimension)            
            cbounds.setLow(0,-max((BoundaryMaxMin[1]-BoundaryMaxMin[0]),(BoundaryMaxMin[3]-BoundaryMaxMin[2]))/25)
            cbounds.setHigh(0,max((BoundaryMaxMin[1]-BoundaryMaxMin[0]),(BoundaryMaxMin[3]-BoundaryMaxMin[2]))/25)
            cbounds.setLow(1,-pi/5)
            cbounds.setHigh(1,pi/5)
            cspace.setBounds(cbounds) 
            
            if self.system_print == True:
                print cspace.settings()
            
           
        if self.Geometric_Control == 'G':
            # define a simple setup class
            ss = og.SimpleSetup(space)
            
        else:
            # define a simple setup class
            ss = oc.SimpleSetup(cspace)
            # set state validity checking for this space
            ss.setStatePropagator(oc.StatePropagatorFn(self.propagate))
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid)) 
        
        
        # create a start state
        start = ob.State(space)
        pose = self.pose_handler.getPose()  #x,y,w,(z if using ROS quadrotor)
        if not self.currentRegionPoly.covers(PolyShapes.Circle(self.radius,(pose[0],pose[1]))):
            print "going to use last goal state as the start state."
            pose[0] = (self.OMPLpath.getSolutionPath().getState(self.OMPLpath.getSolutionPath().getStateCount()-1)).getX()
            pose[1] = (self.OMPLpath.getSolutionPath().getState(self.OMPLpath.getSolutionPath().getStateCount()-1)).getY()
        start().setX(pose[0]) 
        start().setY(pose[1])
        if self.Space_Dimension == 2:
            start().setYaw(0) #pose[2]
        else:
            start().setZ(1.5)            ############ CHANGE!!!!
            #start().setZ(pose[3])
            start().rotation().setIdentity()
        """
        start[0] = pose[0]
        start[1] = pose[1]
        """
        print start()
        
        # create goal states
        
        print goalPoints
        goalStates = ob.GoalStates(ss.getSpaceInformation())
        for i in range(shape(goalPoints)[1]):
            goal = ob.State(space)
            goal().setX(goalPoints[0,i])
            goal().setY(goalPoints[1,i])
            if self.Space_Dimension == 3:
                """
                # pick a random height 
                z_goalPoint = random.uniform(0+self.height, self.maxHeight-self.height)
                goal().setZ(z_goalPoint)           
                #goal().setZ(self.maxHeight/2)
                """
                print current_region,next_region
                print self.map['height'][current_region],self.map['height'][next_region]
                z_goalPoint = min(self.map['height'][current_region]/2,self.map['height'][next_region]/2)
                goal().setZ(z_goalPoint)
                goal().rotation().setIdentity()
            else:
                goal().setYaw(0.0)
            """
            goal[0] = goalPoints[0,i]
            goal[1] = goalPoints[1,i]
            """
            if self.plotting == True:
                if self.Space_Dimension == 3:
                    self.ax.plot([goalPoints[0,i]],[goalPoints[1,i]],[z_goalPoint],'ro')
                else:
                    self.ax.plot([goalPoints[0,i]],[goalPoints[1,i]],'ro')
                self.setPlotLimitXYZ()
                self.ax.get_figure().canvas.draw()
                
            goalStates.addState(goal)
        print goalStates
        
        """
        goalStates = ob.GoalStates(ss.getSpaceInformation())
        NextRegionCenter = self.nextRegionPoly.center()
        # check if the robot is covered by the next reion when it stands in the middle of the region
        if self.nextRegionPoly.covers(PolyShapes.Circle(self.radius,NextRegionCenter)):
            setAsGoal = NextRegionCenter
            if self.system_print == True:
                print "Goal point is the center of the next region"
        else:   # find a random point in the region that the robot can be covered by when it stands there
            goalPointInsideNextRegion = False
            while not goalPointInsideNextRegion:
                sample = self.nextRegionPoly.sample(random.random)
                if self.nextRegionPoly.covers(PolyShapes.Circle(self.radius,sample)):
                    setAsGoal = sample
                    goalPointInsideNextRegion = True
                    if self.system_print == True:
                        print "Goal point is a random point inside the next region: " + str(setAsGoal) 
        goal = ob.State(space)
        goal[0] = setAsGoal[0]
        goal[1] = setAsGoal[1]
        goalStates.addState(goal)   
        """


        # set the start and goal states;
        #ss.setStartAndGoalStates(start, goal)
        ss.setGoal(goalStates)
        ss.setStartState(start)

        # set sampler (optional; the default is uniform sampling)
        si = ss.getSpaceInformation()
        
        # set planner
        planner_prep = self.planner_dictionary[self.Geometric_Control][self.planner]
        planner = planner_prep(si)
        ss.setPlanner(planner)
        
        if self.Geometric_Control == 'G':            
            if not self.planner == 'PRM':
                #pass
                planner.setRange(self.radius*2)
                print "planner.getRange():" + str(planner.getRange())
                #if not self.planner == 'RRTConnect':
                #    planner.setGoalBias(0.5)
            
        else:
            # (optionally) set propagation step size
            si.setPropagationStepSize(1)  #actually is the duration in propagate
            #si.setMinMaxControlDuration(3,3) # is the no of steps taken with the same velocity and omega
            print "radius: " +str(self.radius)
            print "si.getPropagationStepSize():" + str(si.getPropagationStepSize())  
            #print "si.getMinControlDuration():" + str(si.getMinControlDuration())  
            #print "si.getMaxControlDuration():" + str(si.getMaxControlDuration())       
            planner.setGoalBias(0.5)
        ss.setup()
        

        # attempt to solve the problem within ten seconds of planning time
        solved = ss.solve(1000.0) #10
        
        if (solved):
            print("Found solution:")
            # print the path to screen
            print >>sys.__stdout__,(ss.getSolutionPath())
            #print(ss.getSolutionPath().getStates())
            #print(ss.getSolutionPath().getState(0)).getX()
            #print(ss.getSolutionPath().getState(0))[0]
            #print(ss.getSolutionPath().getState(0))[1]
            #print(ss.getSolutionPath().getStateCount())
            #PlannerData = ob.PlannerData(si)
            #print "get planner data: " + str(ss.getPlannerData(PlannerData))
        else:
            print("No solution found")
        
        
        if self.plotting == True :
            self.ax.set_title('Map with Geo/Control: '+str(self.Geometric_Control) + ",Planner:" +str(self.planner), fontsize=12)
            self.ax.set_xlabel('x')
            self.ax.set_ylabel('y')
            for i in range(ss.getSolutionPath().getStateCount()-1):
                #print i
    
                """            
                plt.plot(((ss.getSolutionPath().getStates())[i][0],(ss.getSolutionPath().getStates())[i+1][0]),((ss.getSolutionPath().getStates())[i][1],(ss.getSolutionPath().getStates())[i+1][1]),'b')
                plt.figure(1).canvas.draw()
                """
                if self.Space_Dimension == 3:
                    self.ax.plot(((ss.getSolutionPath().getState(i)).getX(),(ss.getSolutionPath().getState(i+1)).getX()),((ss.getSolutionPath().getState(i)).getY(),(ss.getSolutionPath().getState(i+1)).getY()),((ss.getSolutionPath().getState(i)).getZ(),(ss.getSolutionPath().getState(i+1)).getZ()),'b')
                    self.ax.text((ss.getSolutionPath().getState(i)).getX(),(ss.getSolutionPath().getState(i)).getY(),(ss.getSolutionPath().getState(i)).getZ(), i,fontsize=12)
                    
                else:
                    self.ax.plot(((ss.getSolutionPath().getState(i)).getX(),(ss.getSolutionPath().getState(i+1)).getX()),((ss.getSolutionPath().getState(i)).getY(),(ss.getSolutionPath().getState(i+1)).getY()),0,'b')
                    self.ax.text((ss.getSolutionPath().getState(i)).getX(),(ss.getSolutionPath().getState(i)).getY(),0, i,fontsize=12)
                
            #print [(ss.getSolutionPath().getState(ss.getSolutionPath().getStateCount()-1)).getX()]
            if self.Space_Dimension == 3:
                self.ax.text((ss.getSolutionPath().getState(ss.getSolutionPath().getStateCount()-1)).getX(),(ss.getSolutionPath().getState(ss.getSolutionPath().getStateCount()-1)).getY(),(ss.getSolutionPath().getState(ss.getSolutionPath().getStateCount()-1)).getZ(), ss.getSolutionPath().getStateCount()-1, fontsize=12)
            else:
                self.ax.text((ss.getSolutionPath().getState(ss.getSolutionPath().getStateCount()-1)).getX(),(ss.getSolutionPath().getState(ss.getSolutionPath().getStateCount()-1)).getY(),0, ss.getSolutionPath().getStateCount()-1, fontsize=12)
            self.setPlotLimitXYZ()
            #self.ax.get_figure().canvas.draw()
        
        #print ss
        return ss

class _MyTkApp(threading.Thread):
    def __init__(self):
        self.fig = Figure(figsize=(5,4), dpi=100)
        self.ax = Axes3D(self.fig)
        threading.Thread.__init__(self)
        
    def getSharedData(self):
		# A dictionary of any objects that will need to be shared with other handlers
		return self    
		
    def _quit(self):
        self.root.quit()     # stops mainloop
        self.root.destroy()  # this is necessary on Windows to prevent
        
    def run(self):
        self.root=Tk.Tk() #root
        self.root.wm_title("Embedding in TK")
     
        #self.ax = self.fig.gca(projection='3d')
        #t = arange(0.0,3.0,0.01)
        #s = sin(2*pi*t)
        #self.ax.plot(t,s,1)
        
        # a tk.DrawingArea
        canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.ax.mouse_init()
        canvas.show()
        canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        toolbar = NavigationToolbar2TkAgg( canvas, self.root )
        toolbar.update()
        canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
        
        button = Tk.Button(master=self.root, text='Quit', command=self._quit)
        button.pack(side=Tk.BOTTOM)
        #self.s = Tkinter.StringVar()
        #self.s.set('Foo')
        #l = Tkinter.Label(self.root,textvariable=self.s)
        #l.pack()
        self.root.mainloop()

                    
## @cond IGNORE

# This is a problem-specific sampler that automatically generates valid
# states; it doesn't need to call SpaceInformation::isValid. This is an
# example of constrained sampling. If you can explicitly describe the set valid
# states and can draw samples from it, then this is typically much more
# efficient than generating random samples from the entire state space and
# checking for validity.
class _MyValidStateSampler(ob.ValidStateSampler):
    def __init__(self, si):
        super(_MyValidStateSampler, self).__init__(si)
        self.name_ = "my sampler"
        self.rng_ = ou.RNG()

    # Generate a sample in the valid part of the R^2 state space.
    # Valid states satisfy the following constraints:
    # -1<= x,y,z <=1
    # if .25 <= z <= .5, then |x|>.8 and |y|>.8
    def sample(self, state):
        z = self.rng_.uniformReal(-1,1)

        if z>.25 and z<.5:
            x = self.rng_.uniformReal(0,1.8)
            y = self.rng_.uniformReal(0,.2)
            i = self.rng_.uniformInt(0,3)
            if i==0:
                state[0]=x-1
                state[1]=y-1
            elif i==1:
                state[0]=x-.8
                state[1]=y+.8
            elif i==2:
                state[0]=y-1
                state[1]=x-1
            elif i==3:
                state[0]=y+.8
                state[1]=x-.8
        else:
            state[0] = self.rng_.uniformReal(-1,1)
            state[1] = self.rng_.uniformReal(-1,1)
        state[2] = z
        return True

if __name__ == '__main__':
    print("Using default uniform sampler:")
    motionControlHandler.plan(0)
    print("\nUsing obstacle-based sampler:")
    plan(1)
    print("\nUsing my sampler:")
    plan(2)
