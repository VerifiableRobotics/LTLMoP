#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
===================================================================
BugController.py - Bug Algorithm Motion Controller
===================================================================

This motion controller uses the Bug 2 algorithm developed by V. Lumelsky and A. Stepanov. The algorithm assumes the robot is a
point operating in the plane with a contact sensor or a zero range sensor to
detect obstacles. When the robot has a ﬁnite range (non-zero range) sensor
"""


#import  BugControllerHelper
from numpy import *
from __is_inside import is_inside
import Polygon,Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes
import time, math
import sys,os
import matplotlib.pyplot as plt
import copy
from scipy.linalg import norm
from math import *
import random
import matplotlib.animation as animation
import thread
import threading


import lib.handlers.handlerTemplates as handlerTemplates

class BugControllerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data,robot_type):
        """
        Bug alogorithm motion planning controller

        robot_type (int): Which robot is used for execution. pioneer is 1, ODE is 2 (default=1)
        """
        self.velocity_count_thres = 100;
        self.velocity_count = 0;

        #operate_system (int): Which operating system is used for execution. Ubuntu and Mac is 1, Windows is 2
        if sys.platform in ['win32', 'cygwin']:
            self.operate_system = 2
        else:
            self.operate_system = 1

        # Information about the robot
        ## 1: Pioneer ; 2: 0DE
        if robot_type not in [1,2]:
            robot_type = 1
        self.system = robot_type

        #settings for Ubuntu or Mac plotting (only when you set operate_system = 1)
        self.PLOT              = False    # plot with matplot
        self.PLOT_M_LINE       = False    # plot m-line
        self.PLOT_EXIT         = False    # plot exit point of a region
        self.PLOT_OVERLAP      = False    # plot overlap area with the obstacle

        #settings for Windows (only when you set operate_system = 2)
        self.PLOT_WINDOWS     = True

        # Get references to handlers we'll need to communicate with
        self.drive_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler)
        self.pose_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler) 
        if self.system == 1:
            self.robocomm = shared_data['robocomm']

        # Get information about regions
        self.proj = executor.proj
        self.coordmap_map2lab = self.hsub.coordmap_map2lab
        self.coordmap_lab2map = self.hsub.coordmap_lab2map
        self.last_warning = 0



        ###################################
        ########used by Bug algorithm######
        ###################################

        # PARAMETERS

        #for plotting
        self.time = time.clock()
        self.time_thres = 1;

        # Pioneer related parameters
        self.PioneerWidthHalf  = 0.20     #0.25 # (m) width of Pioneer    #0.20
        self.PioneerLengthHalf = 0.25     #0.30 (m) lenght of Pioneer   #0.25

        # Real Robot polygon related parameters
        self.boxRealVertical         = self.PioneerLengthHalf*2
        self.boxRealHorizontal       = self.PioneerWidthHalf*2.5
        self.boxRealVertical_shift   = self.boxRealVertical/2
        self.boxRealHorizontal_shift = self.boxRealHorizontal/2

        # Pioneer Range related parameters
        self.range             = 2*self.PioneerLengthHalf+0.40     # (m) specify the range of the robot (when the normal circle range cannot detect obstacle)   #0.85
        self.obsRange          = self.range*0.7                                 # (m) range that says the robot detects obstacles    #0.25
        self.shift             = 0.20                                           # How far the range is shifted to ensure it is sensing region in front is bigger    0.20
        self.boxVertical       = self.obsRange*2                                # box cutting from range of Pioneer
        self.boxHorizontal     = self.obsRange*2                                 # box cutting from range of Pioneer
        self.boxVertical_shift = self.boxVertical + self.boxRealVertical/2*1.5    # vertical shifting of box
        self.boxHorizontal_shift = self.boxHorizontal/2                          # horizontal shifting of the box

        ## 2: 0DE
        self.factorODE = 30    # 30 works better than 50
        if  self.system == 2:
            self.PioneerWidthHalf       = self.PioneerWidthHalf*self.factorODE
            self.PioneerLengthHalf      = self.PioneerLengthHalf*self.factorODE
            self.range                  = self.range*self.factorODE
            self.obsRange               = self.obsRange*self.factorODE
            self.shift                  = self.shift*self.factorODE
            self.boxVertical            = self.boxVertical*self.factorODE
            self.boxHorizontal          = self.boxHorizontal*self.factorODE
            self.boxVertical_shift      = self.boxVertical_shift*self.factorODE
            self.boxHorizontal_shift    = self.boxHorizontal_shift*self.factorODE
            self.boxRealVertical        = self.boxRealVertical*self.factorODE
            self.boxRealHorizontal      = self.boxRealHorizontal*self.factorODE
            self.boxRealVertical_shift  = self.boxRealVertical_shift*self.factorODE
            self.boxRealHorizontal_shift= self.boxRealHorizontal_shift*self.factorODE

        self.map = {}                             # dictionary for all the regions
        self.all = Polygon.Polygon()              # Polygon with all the regions
        self.map_work = Polygon.Polygon()         # Polygon of the current region and next region considered
        self.ogr = Polygon.Polygon()              #Polygon built from occupancy grid data points

        self.previous_current_reg = None    # previous current region
        self.currentRegionPoly  = None      # current region's polygon
        self.nextRegionPoly    = None       # next region's polygon
        self.overlap           = None
        self.q_g               = [0,0]      # goal point of the robot heading to
        self.q_hit             = [0,0]      # location where the robot first detect an obstacle
        self.boundary_following= False      # tracking whether it is in boundary following mode
        self.m_line            = None       # m-line polygon
        self.trans_matrix      = mat([[0,1],[-1,0]])   # transformation matrix for find the normal to the vector connecting a point on the obstacle to the robot
        self.q_hit_count       = 0
        self.q_hit_Thres       = 1000
        self.prev_follow       = [[],[]]


        ## Construct robot polygon (for checking overlap)
        pose = self.pose_handler.getPose()
        self.prev_pose = pose
        self.robot = PolyShapes.Rectangle(self.boxHorizontal,self.boxVertical)
        self.robot.shift(pose[0]-self.boxHorizontal_shift,pose[1]-self.boxVertical_shift)
        self.robot = PolyShapes.Circle(self.obsRange,(pose[0],pose[1])) - self.robot
        self.robot.rotate(pose[2]-pi/2,pose[0],pose[1])
        self.robot.shift(self.shift*cos(pose[2]),self.shift*sin(pose[2]))

        #construct real robot polygon( see if there is overlaping with path to goal
        self.realRobot = PolyShapes.Rectangle(self.boxRealHorizontal,self.boxRealVertical )
        self.realRobot.shift(pose[0]-self.boxRealHorizontal_shift,pose[1]-self.boxRealVertical_shift)
        self.realRobot.rotate(pose[2]-pi/2,pose[0],pose[1])

        #constructing polygon of different regions (holes being taken care)

        for region in self.proj.rfi.regions:
            self.map[region.name] = self.createRegionPolygon(region)
            for n in range(len(region.holeList)): # no of holes
                self.map[region.name] -= self.createRegionPolygon(region,n)

        #construct a polygon that included all the regions
        for regionName,regionPoly in self.map.iteritems():
            self.all += regionPoly


        #setting for plotting
        if self.operate_system == 1:
            if self.PLOT or self.PLOT_OVERLAP == True:
                self.original_figure = 1
                plt.figure(self.original_figure)

            if self.PLOT_EXIT or self.PLOT_M_LINE == True:
                self.overlap_figure  = 2
                plt.figure(self.overlap_figure)

        else:
            if self.PLOT_WINDOWS == True:
                # start using anmination to plot Pioneer
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111)
                self.scope = _Scope(self.ax,self)
                thread.start_new_thread(self.jplot,())

        #Plot the robot on the map in figure 1
        if self.PLOT == True:
            plt.figure(self.original_figure)
            plt.clf()
            self.plotPoly(self.realRobot, 'r')
            self.plotPoly(self.robot, 'b')
            plt.plot(pose[0],pose[1],'bo')
            self.plotPioneer(self.original_figure)

    def gotoRegion(self, current_reg, next_reg, last=False):

        """
        If ``last`` is True, we will move to the center of the destination region.
        Returns ``True`` if we've reached the destination region.
        """
        q_gBundle              = [[],[]]          # goal bundle
        q_overlap              = [[],[]]          # overlapping points with robot range
        pose = self.pose_handler.getPose()        # Find our current configuration

        #Plot the robot on the map in figure 1
        if self.PLOT == True:
            plt.figure(self.original_figure)
            plt.clf()
            self.plotPoly(self.realRobot, 'r')
            self.plotPoly(self.robot, 'b')
            plt.plot(pose[0],pose[1],'bo')
            self.plotPioneer(self.original_figure)


        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return False

        # Check if Vicon has cut out
        if math.isnan(pose[2]):
            print "no vicon pose"
            print "WARNING: No Vicon data! Pausing."
            #self.drive_handler.setVelocity(0, 0)  # So let's stop
            time.sleep(1)
            return False

        ###This part is run when the robot goes to a new region, otherwise, the original map will be used.
        if not self.previous_current_reg == current_reg:
            #print 'getting into bug alogorithm'

            #clean up the previous self.map_work
            self.map_work =  Polygon.Polygon()

            # NOTE: Information about region geometry can be found in self.proj.rfi.regions
            # create polygon list for regions other than the current_reg and the next_reg
            self.map_work += self.map[self.proj.rfi.regions[current_reg].name]
            self.map_work += self.map[self.proj.rfi.regions[next_reg].name]

            # building current polygon and destination polygon
            self.nextRegionPoly    = self.map[self.proj.rfi.regions[next_reg].name]
            self.currentRegionPoly = self.map[self.proj.rfi.regions[current_reg].name]

            #set to zero velocity before finding the tranFace
            self.drive_handler.setVelocity(0, 0)
            if last:
                transFace = None
            else:
                print "Current reg is " + str(self.proj.rfi.regions[current_reg].name.lower())
                print "Next reg is "+ str(self.proj.rfi.regions[next_reg].name.lower())


                for i in range(len(self.proj.rfi.transitions[current_reg][next_reg])):
                    pointArray_transface = [x for x in self.proj.rfi.transitions[current_reg][next_reg][i]]
                    transFace = asarray(map(self.coordmap_map2lab,pointArray_transface))
                    bundle_x = (transFace[0,0] +transFace[1,0])/2    #mid-point coordinate x
                    bundle_y = (transFace[0,1] +transFace[1,1])/2    #mid-point coordinate y
                    q_gBundle = hstack((q_gBundle,vstack((bundle_x,bundle_y))))
                q_gBundle = q_gBundle.transpose()

                # Find the closest face to the current position
                max_magsq = 1000000
                for tf in q_gBundle:
                    magsq = (tf[0] - pose[0])**2 + (tf[1] - pose[1])**2
                    if magsq < max_magsq:
                        connection = 0
                        tf = tf+(tf-asarray(self.currentRegionPoly.center()))/norm(tf-asarray(self.currentRegionPoly.center()))*2.1*self.PioneerLengthHalf
                        if not self.nextRegionPoly.covers(PolyShapes.Circle(self.PioneerLengthHalf*2,(tf[0],tf[1]))):
                            tf = tf-(tf-asarray(self.currentRegionPoly.center()))/norm(tf-asarray(self.currentRegionPoly.center()))*4.2*self.PioneerLengthHalf
                            if self.nextRegionPoly.covers(PolyShapes.Circle(self.PioneerLengthHalf*2,(tf[0],tf[1]))):
                                connection = 1
                        else:
                            connection = 1
                        if connection == 1:
                            pt1  = tf
                            max_magsq = magsq
                            transFace = 1
                            self.q_g[0] = pt1[0]
                            self.q_g[1] = pt1[1]
                        else:
                            sample = False
                            while not sample:
                                self.q_g[0],self.q_g[1] = self.nextRegionPoly.sample(random.random)
                                robo = PolyShapes.Circle(self.PioneerLengthHalf,(self.q_g[0],self.q_g[1]))
                                if not bool(robo - self.nextRegionPoly):
                                    sample = True


                """
                # Push the goal point to somewhere inside the next region to ensure the robot will get there.(CHECK!!)
                self.q_g = self.q_g+(self.q_g-asarray(self.currentRegionPoly.center()))/norm(self.q_g-asarray(self.currentRegionPoly.center()))*3*self.PioneerLengthHalf
                if not self.nextRegionPoly.isInside(self.q_g[0],self.q_g[1]):
                    self.q_g = self.q_g-(self.q_g-asarray(self.currentRegionPoly.center()))/norm(self.q_g-asarray(self.currentRegionPoly.center()))*6*self.PioneerLengthHalf
                """

                #plot exiting point
                if self.PLOT_EXIT == True:
                    plt.figure(self.overlap_figure)
                    plt.clf()
                    plt.plot(q_gBundle[:,0],q_gBundle[:,1],'ko' )
                    plt.plot(self.q_g[0],self.q_g[1],'ro')
                    plt.plot(pose[0],pose[1],'bo')
                    self.plotPioneer(self.overlap_figure,0)

                if transFace is None:
                    print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.proj.rfi.regions[current_reg].name, self.proj.rfi.regions[next_reg].name)


        ##################################################
        #######check whether obstacle is detected#########
        ##################################################

        #Update pose,update self.robot, self.realRobot orientation
        self.robot.shift(pose[0]-self.prev_pose[0],pose[1]-self.prev_pose[1])
        self.realRobot.shift(pose[0]-self.prev_pose[0],pose[1]-self.prev_pose[1])
        self.robot.rotate(pose[2]-self.prev_pose[2],pose[0],pose[1])
        self.realRobot.rotate(pose[2]-self.prev_pose[2],pose[0],pose[1])
        self.prev_pose = pose

        ############################
        ########### STEP 1##########
        ############################
        ##Check whether obsRange overlaps with obstacle or the boundary (overlap returns the part of robot not covered by the region

        # for real Pioneer robot
        if self.system == 1:
            # motion controller is not in boundary following mode
            if self.boundary_following == False:

                if self.robocomm.getReceiveObs() == False:
                    overlap = self.robot - ( self.map_work)
                else:
                    overlap = self.robot  - ( self.map_work - self.robocomm.getObsPoly())

            else: #TRUE
                # use a robot with full range all around it
                Robot = PolyShapes.Circle(self.obsRange,(pose[0],pose[1]))
                Robot.shift(self.shift*cos(pose[2]),self.shift*sin(pose[2]))

                if self.robocomm.getReceiveObs() == False:
                    overlap = Robot - ( self.map_work)
                else:
                    overlap = Robot  - ( self.map_work - self.robocomm.getObsPoly())
        # for ODE
        else:
            if self.boundary_following == False:
                overlap = self.robot  - (self.map_work)
            else:#TRUE
                overlap = self.robot  - (self.map_work)

        if self.boundary_following == False:
            if bool(overlap):    ## overlap of obstacles
                #print "There MAYBE overlap~~ check connection to goal"

                # check whether the real robot or and path to goal overlap with the obstacle
                QGoalPoly= PolyShapes.Circle(self.PioneerLengthHalf,(self.q_g[0],self.q_g[1]))
                path  = PolyUtils.convexHull(self.realRobot + QGoalPoly)
                if self.system == 1:
                    if self.robocomm.getReceiveObs() == False:
                        pathOverlap = path - ( self.map_work)
                    else:
                        pathOverlap = path  - ( self.map_work - self.robocomm.getObsPoly())
                else:
                    pathOverlap = path - ( self.map_work)


                if bool(pathOverlap):   # there is overlapping, go into bounding following mode
                    #print "There IS overlap"
                    self.q_hit  = mat([pose[0],pose[1]]).T
                    self.boundary_following = True

                    #Generate m-line polygon
                    QHitPoly = PolyShapes.Circle(self.PioneerLengthHalf/4,(pose[0],pose[1]))
                    QGoalPoly= PolyShapes.Circle(self.PioneerLengthHalf/4,(self.q_g[0],self.q_g[1]))
                    self.m_line  = PolyUtils.convexHull(QHitPoly + QGoalPoly)

                    #plot the first overlap
                    if self.PLOT_M_LINE == True:
                        plt.figure(self.overlap_figure)
                        plt.clf()
                        self.plotPoly(QHitPoly,'k')
                        self.plotPoly(QGoalPoly,'k')
                        self.plotPoly(overlap,'g')
                        self.plotPoly(self.m_line,'b')
                        plt.plot(pose[0],pose[1],'bo')
                        self.plotPioneer(self.overlap_figure,0)


                else:                ##head towards the q_goal
                    if self.system == 1:
                        if self.robocomm.getReceiveObs() == False:   # wait for obstacles from Pioneer
                            vx = 0
                            vy = 0
                        else:
                            dis_cur  = vstack((self.q_g[0],self.q_g[1]))- mat([pose[0],pose[1]]).T
                            vx = (dis_cur/norm(dis_cur)/3)[0,0]
                            vy = (dis_cur/norm(dis_cur)/3)[1,0]

                    else:
                        dis_cur  = vstack((self.q_g[0],self.q_g[1]))- mat([pose[0],pose[1]]).T
                        vx = (dis_cur/norm(dis_cur)/3)[0,0]
                        vy = (dis_cur/norm(dis_cur)/3)[1,0]
                        #print "no obstacles 2-ODE true"

            else:                ##head towards the q_goal
                    if self.system == 1:
                        if self.robocomm.getReceiveObs() == False:   # wait for obstacles from Pioneer
                            vx = 0
                            vy = 0
                        else:
                            dis_cur  = vstack((self.q_g[0],self.q_g[1]))- mat([pose[0],pose[1]]).T
                            vx = (dis_cur/norm(dis_cur)/3)[0,0]
                            vy = (dis_cur/norm(dis_cur)/3)[1,0]

                    else:
                        dis_cur  = vstack((self.q_g[0],self.q_g[1]))- mat([pose[0],pose[1]]).T
                        vx = (dis_cur/norm(dis_cur)/3)[0,0]
                        vy = (dis_cur/norm(dis_cur)/3)[1,0]
                        #print "no obstacles 1-ODE true"

        if self.boundary_following == True:
            self.q_hit_count       += 1
            # finding the point to go normal to (closest overlapping point)

            j = 0
            recheck = 0
            while not bool(overlap):
                # cannot see the obstacle. Based on the location of the previous point blow up the range of the robot on the left or on the right
                j += 1

                # finding whether the previous obstacle point is on the left side or the right side of the robot
                # angle = angle of the previous point from the x-axis of the field
                # omega = angle of the current Pioneer orientation from the x-axis of the field
                # cc    = differnece between angle and omega ( < pi = previous point on the left of robot, else on the right of robot)
                x = self.prev_follow[0] -pose[0]
                y = self.prev_follow[1] -pose[1]
                angle = atan(y/x)

                # convert angle to 2pi
                if x > 0 and y > 0:
                    angle = angle
                elif x < 0 and y > 0:
                    angle = pi + angle
                elif x <0 and y < 0:
                    angle = pi + angle
                else:
                    angle = 2*pi + angle

                # convert pose to 2pi
                if pose[2] < 0:
                    omega = (2*pi + pose[2])
                else:
                    omega = pose[2]


                if omega > angle:
                    cc = 2*pi - (omega - angle)
                else:
                    cc = angle - omega

                # on the left
                #if angle - omega > 0 and angle - omega < pi:
                if cc < pi:
                    #print "on the left, angle: "+ str(angle) + " omega: "+ str(omega)+ " angle-omega: "+ str(angle-omega)
                    Robot = PolyShapes.Rectangle(self.range*2*j,self.range*2*j)
                    Robot.shift(pose[0]-self.range*j*2,pose[1]-self.range*j)
                    Robot.rotate(pose[2]-pi/2,pose[0],pose[1])

                # on the right
                else:
                    #print "on the right, angle: "+ str(angle) + " omega: "+ str(omega)+ " angle-omega: "+ str(angle-omega)
                    Robot = PolyShapes.Rectangle(self.range*2*j,self.range*2*j)
                    Robot.shift(pose[0],pose[1]-self.range*j)
                    Robot.rotate(pose[2]-pi/2,pose[0],pose[1])

                if self.system == 1:
                    overlap = Robot - ( self.map_work - self.robocomm.getObsPoly())
                else:
                    overlap = Robot - ( self.map_work)

                #self.plotPoly(Robot, 'm',2)
                #determines as dynamic obstacles and can be go striaight to the goal point
                if j >= 2:
                    dis_cur  = vstack((self.q_g[0],self.q_g[1]))- mat([pose[0],pose[1]]).T
                    vx = (dis_cur/norm(dis_cur)/3)[0,0]
                    vy = (dis_cur/norm(dis_cur)/3)[1,0]
                    overlap = None
                    self.overlap = overlap
                    self.q_hit_count        = 0
                    self.boundary_following = False
                    self.m_line             = None
                    self.drive_handler.setVelocity(vx,vy, pose[2])
                    RobotPoly = PolyShapes.Circle(self.PioneerLengthHalf+0.06,(pose[0],pose[1]))     ###0.05
                    departed = not self.currentRegionPoly.overlaps(self.realRobot)
                    #departed = not self.currentRegionPoly.overlaps(self.realRobot) and (not (self.nextRegionPoly.overlaps(self.realRobot) and not self.nextRegionPoly.covers(self.realRobot)))
                    arrived  = self.nextRegionPoly.covers(self.realRobot)
                    return arrived




            ##extra box plotting in figure 1#
            if self.PLOT_OVERLAP == True:
                plt.figure(self.original_figure)
                plt.clf()
                self.plotPoly(self.realRobot, 'r')
                self.plotPoly(self.robot, 'b')
                self.plotPoly(overlap,'g',3)
                plt.plot(pose[0],pose[1],'bo')
                self.plotPioneer(self.original_figure)

            # find the closest point on the obstacle to the robot
            overlap_len = len(overlap)
            for j in range(overlap_len):
                BoundPolyPoints = asarray(overlap[j])
                for i in range(len(BoundPolyPoints)-1):
                    bundle_x = (BoundPolyPoints[i,0] +BoundPolyPoints[1+i,0])/2    #mid-point coordinate x
                    bundle_y = (BoundPolyPoints[i,1] +BoundPolyPoints[1+i,1])/2    #mid-point coordinate y
                    q_overlap = hstack((q_overlap,vstack((bundle_x,bundle_y))))
                bundle_x = (BoundPolyPoints[len(BoundPolyPoints)-1,0] +BoundPolyPoints[0,0])/2    #mid-point coordinate x
                bundle_y = (BoundPolyPoints[len(BoundPolyPoints)-1,1] +BoundPolyPoints[0,1])/2    #mid-point coordinate y
                q_overlap = hstack((q_overlap,vstack((bundle_x,bundle_y))))
            q_overlap = q_overlap.transpose()
            pt =  self.closest_pt([pose[0],pose[1]], vstack((q_overlap,asarray(PolyUtils.pointList(overlap)))))
            self.prev_follow = pt

            #calculate the vector to follow the obstacle
            normal = mat([pose[0],pose[1]] - pt)

            #find the distance from the closest point
            distance = norm(normal)
            velocity = normal * self.trans_matrix
            vx = (velocity/norm(velocity)/3)[0,0]
            vy = (velocity/norm(velocity)/3)[0,1]

            # push or pull the robot towards the obstacle depending on whether the robot is close or far from the obstacle.
            turn = pi/4*(distance-0.5*self.obsRange)/(self.obsRange)    ### change to 0.6 from 0.5 for more allowance in following
            corr_matrix       = mat([[cos(turn),-sin(turn)],[sin(turn),cos(turn)]])
            v =  corr_matrix*mat([[vx],[vy]])
            vx = v[0,0]
            vy = v[1,0]


            ##plotting overlap on figure 2
            if self.PLOT_OVERLAP == True:
                plt.figure(self.overlap_figure)
                plt.clf()
                self.plotPoly(self.m_line,'b');
                self.plotPoly(overlap,'r');
                plt.plot(pt[0],pt[1],'ro')
                plt.plot(pose[0],pose[1],'bo')
                self.plotPioneer(self.overlap_figure,0)


            ## conditions that the loop will end
            #for 11111
            RobotPoly = PolyShapes.Circle(self.PioneerLengthHalf+0.06,(pose[0],pose[1]))   ####0.05
            departed = not self.currentRegionPoly.overlaps(self.realRobot)
            #departed = not self.currentRegionPoly.overlaps(self.realRobot) and (not (self.nextRegionPoly.overlaps(self.realRobot) and not self.nextRegionPoly.covers(self.realRobot)))
            arrived  = self.nextRegionPoly.covers(self.realRobot)

            #for 33333
            reachMLine= self.m_line.overlaps(RobotPoly)

            # 1.reached the next region
            if arrived:
                self.boundary_following = False
                self.m_line             = None
                self.q_hit_count       = 0
                print "arriving at the next region. Exit boundary following mode"
                vx = 0
                vy = 0

                """
                # 2.q_hit is reencountered
                elif norm(self.q_hit-mat([pose[0],pose[1]]).T) < 0.05 and self.q_hit_count > self.q_hit_Thres:
                    print "reencounter q_hit. cannot reach q_goal"
                    vx = 0
                    vy = 0
                """

            # 3.m-line reencountered
            elif reachMLine:
                #print >>sys.__stdout__, "m-line overlaps RoboPoly, m-line" + str(norm(self.q_g-self.q_hit)-2*self.obsRange) + " distance: " + str(norm(self.q_g-mat([pose[0],pose[1]]).T))
                if norm(self.q_g-mat([pose[0],pose[1]]).T) < norm(self.q_g-self.q_hit)-2*self.obsRange:
                    #print "m-line overlaps RoboPoly, m-line" + str(norm(self.q_g-self.q_hit)-2*self.obsRange) + " distance: " + str(norm(self.q_g-mat([pose[0],pose[1]]).T))
                    #print "leaving boundary following mode"
                    self.boundary_following = False
                    self.m_line             = None
                    self.q_hit_count       = 0
                    leaving                = False

                    # turn the robot till it is facing the goal
                    while not leaving:
                        x = self.q_g[0] -self.pose_handler.getPose()[0]
                        y = self.q_g[1] -self.pose_handler.getPose()[1]
                        angle = atan(y/x)
                        if x > 0 and y > 0:
                            angle = angle
                        elif x < 0 and y > 0:
                            angle = pi + angle
                        elif x <0 and y < 0:
                            angle = pi + angle
                        else:
                            angle = 2*pi + angle


                        if self.pose_handler.getPose()[2] < 0:
                            omega = (2*pi + self.pose_handler.getPose()[2])
                            #print >>sys.__stdout__,"omega<0: "+ str(omega)
                        else:
                            omega = self.pose_handler.getPose()[2]
                            #print >>sys.__stdout__,"omega: "+ str(omega)


                        if omega > angle:
                            cc = 2*pi - (omega - angle)
                        else:
                            cc = angle - omega

                        # angle(goal point orientation) on the left of omega(robot orientation)
                        #if angle - omega > 0 and angle - omega < pi:
                        if cc < pi:
                            #print>>sys.__stdout__, "turn left"
                            vx,vy = self.turnLeft(cc)

                        # on the right
                        else:
                            #print>>sys.__stdout__, "turn right"
                            vx, vy = self.turnRight(2*pi-cc)

                        #print>>sys.__stdout__, "omega: "+ str(omega) + " angle: "+ str(angle) + " (omega-angle): " + str(omega-angle)
                        self.drive_handler.setVelocity(vx,vy, self.pose_handler.getPose()[2])

                        if omega - angle < pi/6 and omega - angle > -pi/6:
                            leaving = True

            #Check whether the robot can leave now (the robot has to be closer to the goal than when it is at q_hit to leave)
            QGoalPoly= PolyShapes.Circle(self.PioneerLengthHalf,(self.q_g[0],self.q_g[1]))
            path  = PolyUtils.convexHull(self.realRobot + QGoalPoly)
            if self.system == 1:
                if self.robocomm.getReceiveObs() == False:
                    pathOverlap = path - ( self.map_work)
                else:
                    pathOverlap = path  - ( self.map_work - self.robocomm.getObsPoly())
            else:
                pathOverlap = path - ( self.map_work)

            if not bool(pathOverlap):
                #print "There is NO MORE obstacles in front for now."

                # check if the robot is closer to the goal compared with q_hit
                if norm(self.q_hit-mat(self.q_g).T) > norm(mat([pose[0],pose[1]]).T-mat(self.q_g).T) :
                    #print "The robot is closer than the leaving point. The robot can leave"
                    self.boundary_following = False
                    self.m_line             = None
                    self.q_hit_count       = 0
                    dis_cur  = vstack((self.q_g[0],self.q_g[1]))- mat([pose[0],pose[1]]).T
                    vx = (dis_cur/norm(dis_cur)/3)[0,0]
                    vy = (dis_cur/norm(dis_cur)/3)[1,0]
                else:
                    lala = 1
                    #print "not leaving bug algorithm. difference(-farther) =" + str(norm(self.q_hit-mat(self.q_g).T) - norm(mat([pose[0],pose[1]]).T-mat(self.q_g).T))

        """
        # Pass this desired velocity on to the drive handler
        # Check if there are obstacles within 0.35m of the robot, if so, stop the robot
        if self.system == 1:
            if self.robocomm.getSTOP() == True:
                vx = 0
                vy = 0
        """
        #vx = 0
        #vy = 0
        self.overlap = overlap
        self.drive_handler.setVelocity(vx,vy, pose[2])


        # Set the current region as the previous current region(for checking whether the robot has arrived at the next region)
        self.previous_current_reg = current_reg


        # check whether robot has arrived at the next region
        RobotPoly = PolyShapes.Circle(self.PioneerLengthHalf+0.06,(pose[0],pose[1]))     ###0.05
        #departed = not self.currentRegionPoly.overlaps(self.realRobot) and (not (self.nextRegionPoly.overlaps(self.realRobot) and not self.nextRegionPoly.covers(self.realRobot)))
        departed = not self.currentRegionPoly.overlaps(self.realRobot)
        arrived  = self.nextRegionPoly.covers(self.realRobot)
        if arrived:
            self.q_hit_count        = 0
            self.boundary_following = False
            self.m_line             = None

        if departed and (not arrived) and (time.time()-self.last_warning) > 0.5:
            print "WARNING: Left current region but not in expected destination region"
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



    def plotPioneer(self,number,y = 1):
        """
        Plotting regions and obstacles with matplotlib.pyplot

        number: figure number (see on top)
        y = 0 : plot self.map_work instead of self.map
        """
        if self.operate_system == 1:
            if not plt.isinteractive():
                plt.ion()
            plt.hold(True)

        for regionName,regionPoly in self.map.iteritems():
            self.plotPoly(regionPoly,'k')

        if self.system == 1:
            if bool(self.robocomm.getObsPoly()):
                self.plotPoly(self.robocomm.getObsPoly(),'k')

        if self.operate_system == 1:
            plt.figure(number).canvas.draw()

    def turnLeft(self,a):
        """
        Turn left with angle a
        """
        vx = cos(self.pose_handler.getPose()[2]+a)* self.PioneerLengthHalf;
        vy = sin(self.pose_handler.getPose()[2]+a)* self.PioneerLengthHalf;
        return vx,vy

    def turnRight(self,a):
        """
        Turn right with angle a
        """
        vx = cos(self.pose_handler.getPose()[2]-a)* self.PioneerLengthHalf;
        vy = sin(self.pose_handler.getPose()[2]-a)* self.PioneerLengthHalf;
        return vx,vy


    def euclid(self,pt1, pt2):
        """
        for calculating the minimum distance from a point on the obstacle
        """
        pairs = zip(pt1, pt2)                            # Form pairs in corresponding dimensions
        sum_sq_diffs = sum((a - b)**2 for a, b in pairs) # Find sum of squared diff
        return (sum_sq_diffs)**(float(1)/2)              # Take sqrt to get euclidean distance


    def closest_pt(self,pt, vec):
        """
        Returns the point in vec with minimum euclidean distance to pt
        """
        return min(vec, key=lambda x: self.euclid(pt, x))

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

    def getOldRegionName(self, regionName):
        """
        This function returns the old region name (eg: r1, r2, other etc) when given the new region name (eg: p1, p2..)
        """
        for oldRegionName,newRegionNames in self.proj.regionMapping.iteritems():
            if regionName in newRegionNames:
                return oldRegionName
        print 'Cannot find region with sub-region %s' % regionName
        return None

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

    def data_gen(self):
        self.ax.cla()
        self.plotPioneer(1)
        self.plotPoly(self.realRobot, 'r')
        self.plotPoly(self.robot, 'b')
        pose = self.pose_handler.getPose()
        self.ax.plot(pose[0],pose[1],'bo')
        self.ax.plot(self.q_g[0],self.q_g[1],'ro')
        self.plotPoly(self.overlap,'g')
        self.plotPoly(self.m_line,'b')
        yield(pose[0],pose[1])
        self.ax.plot(self.prev_follow[0],self.prev_follow[1],'ko')


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








