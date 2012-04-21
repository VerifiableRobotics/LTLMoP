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
from is_inside import *
import Polygon,Polygon.IO 
from Polygon.Utils import *
from Polygon.Shapes import *
import time, math
import sys,os
import matplotlib.pyplot as plt
import copy
from scipy.linalg import norm
from math import *

class motionControlHandler:
    def __init__(self, proj, shared_data,robot_type):
        """
        Bug alogorithm motion planning controller
        
        robot_type (int): Which robot is used for execution. pioneer is 1, ODE is 2 (default=1)
        """

        # Information about the robot
        ## 1: Pioneer ; 2: 0DE
        if robot_type not in [1,2]:
            robot_type = 1
        self.system = robot_type
                
        #setting for plotting
        ##figure number
        self.original_figure = 1
        self.overlap_figure  = 2
        plt.figure(self.original_figure)
        plt.figure(self.overlap_figure)
        self.PLOT              = True    # plot with matplot
        self.PLOT_M_LINE       = True    # plot m-line
        self.PLOT_EXIT         = True    # plot exit point of a region
        self.PLOT_OVERLAP      = True    # plot overlap area with the obstacle
        
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.h_instance['drive']
        self.pose_handler = proj.h_instance['pose']
        if self.system == 1:
            self.robocomm = shared_data['robocomm']

        # Get information about regions
        self.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab
        self.coordmap_lab2map = proj.coordmap_lab2map
        self.last_warning = 0

                
        ###################################
        ########used by Bug algorithm######
        ###################################



        # PARAMETERS        
        self.LeftRightFlag     = 1       # originally set to right 1- right, 0 left      
        
        # Pioneer related parameters
        self.PioneerWidthHalf  = 0.20     # (m) width of Pioneer
        self.PioneerLengthHalf = 0.25     # (m) lenght of Pioneer
        self.ratioBLOW         = 0.5      # blow up ratio of the pioneer box                             ######### 5 BOX
        self.PioneerBackMargin=  self.ratioBLOW*self.PioneerLengthHalf*2    # (in m)
        
        self.range             = 0.75     # (m) specify the range of the robot (when the normal circle range cannot detect obstacle)
        self.obsRange          = 0.50     # (m) range that says the robot detects obstacles
        self.shift             = 0.20     # 0.15

        
        ## 2: 0DE
        self.factorODE = 50
        if  self.system == 2:
            self.PioneerWidthHalf  = self.PioneerWidthHalf*self.factorODE    
            self.PioneerLengthHalf = self.PioneerLengthHalf*self.factorODE    
            self.range             = self.range*self.factorODE   
            self.obsRange          = self.obsRange*self.factorODE
            self.PioneerBackMargin= self.PioneerBackMargin*self.factorODE
            

        #build self.map with empty contour
        self.map = Rectangle (1,1)   
        self.map -= self.map          #Polygon built from the original map

        #build self.ogr with empty contour
        self.ogr = Rectangle (1,1)   #Polygon built from occupancy grid data points
        self.ogr -= self.ogr
      
        self.freespace = None               # contains only the boundary
        self.map_work  = None               # working copy of the map. remove current region and next region
        self.previous_current_reg = None    # previous current region   
        self.currentRegionPoly  = None      # current region's polygon
        self.nextRegionPoly    = None       # next region's polygon
        self.q_g               = [0,0]      # goal point of the robot heading to 
        self.q_hit             = [0,0]      # location where the robot first detect an obstacle
        self.boundary_following= False      # tracking whether it is in boundary following mode
        self.m_line            = None       # m-line polygon
        self.trans_matrix      = mat([[0,1],[-1,0]])   # transformation matrix for find the normal to the vector connecting a point on the obstacle to the robot
        #self.corr_theta        = pi/6
        #self.corr_matrix       = mat([[cos(self.corr_theta),-sin(self.corr_theta)],[sin(self.corr_theta),cos(self.corr_theta)]])
        self.q_hit_count       = 0
        self.q_hit_Thres       = 1000
        self.prev_follow       = [[],[]]
        
        
        ## Construct robot polygon (for checking overlap)
        pose = self.pose_handler.getPose()
        self.prev_pose = pose        
        self.robot = Rectangle(self.obsRange*3,self.PioneerBackMargin)    
        self.robot.shift(pose[0]-self.obsRange,pose[1]-2*self.PioneerLengthHalf)
        self.robot = Circle(self.obsRange,(pose[0],pose[1])) - self.robot
        self.robot.rotate(pose[2]-pi/2,pose[0],pose[1]) 
        self.robot.shift(self.shift*cos(pose[2]),self.shift*sin(pose[2]))
        
        #construct real robot polygon( see if there is overlaping with path to goal
        self.realRobot = Rectangle(self.PioneerWidthHalf*2.5,self.PioneerLengthHalf*2 )
        self.realRobot.shift(pose[0]-self.PioneerWidthHalf*1.25,pose[1]-self.PioneerLengthHalf*1)
        self.realRobot.rotate(pose[2]-pi/2,pose[0],pose[1]) 
                      
        #constructing map with all the regions included           
        for region in self.rfi.regions: 
            if region.name.lower()=='freespace':
                pointArray = [x for x in region.getPoints()]
                pointArray = map(self.coordmap_map2lab, pointArray)
                regionPoints = [(pt[0],pt[1]) for pt in pointArray]
                freespace    = Polygon(regionPoints)
                freespace_big= Polygon(regionPoints)                
                freespace_big.scale(1.2, 1.2)
                self.freespace = freespace_big -freespace
                self.map += freespace_big -freespace               #without including freespace

            else:
                pointArray = [x for x in region.getPoints()]
                pointArray = map(self.coordmap_map2lab, pointArray)
                regionPoints = [(pt[0],pt[1]) for pt in pointArray]
                self.map +=  Polygon(regionPoints)  

                
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
            print "WARNING: No Vicon data! Pausing."
            #self.drive_handler.setVelocity(0, 0)  # So let's stop
            time.sleep(1)
            return False

        ###This part is run when the robot goes to a new region, otherwise, the original map will be used.       
        if not self.previous_current_reg == current_reg:
            print 'getting into bug alogorithm'
 
            #clean up the previous self.map
            self.map_work =  Polygon(self.map)   
            # create polygon list for regions other than the current_reg and the next_reg. This polygon will be counted as the obstacle if encountered.
            if not self.rfi.regions[current_reg].name.lower() == 'freespace':
                pointArray = [x for x in self.rfi.regions[current_reg].getPoints()]
                pointArray = map(self.coordmap_map2lab, pointArray)
                regionPoints = [(pt[0],pt[1]) for pt in pointArray]
                self.map_work -=  Polygon(regionPoints)
            
            if not self.rfi.regions[next_reg].name.lower() == 'freespace':
                pointArray = [x for x in self.rfi.regions[next_reg].getPoints()]
                pointArray = map(self.coordmap_map2lab, pointArray)
                regionPoints = [(pt[0],pt[1]) for pt in pointArray]
                self.map_work -=  Polygon(regionPoints)

                       
            # NOTE: Information about region geometry can be found in self.rfi.regions
            # building current polygon and destination polygon
            pointArray = [x for x in self.rfi.regions[next_reg].getPoints()]
            pointArray = map(self.coordmap_map2lab, pointArray)
            regionPoints = [(pt[0],pt[1]) for pt in pointArray]
            self.nextRegionPoly = Polygon(regionPoints)
            if self.rfi.regions[next_reg].name.lower()=='freespace':
                for region in self.rfi.regions:
                    if region.name.lower() != 'freespace':
                        pointArray = [x for x in region.getPoints()]
                        pointArray = map(self.coordmap_map2lab, pointArray)
                        regionPoints = [(pt[0],pt[1]) for pt in pointArray]
                        self.nextRegionPoly = self.nextRegionPoly - Polygon(regionPoints)

            pointArray = [x for x in self.rfi.regions[current_reg].getPoints()]
            pointArray = map(self.coordmap_map2lab, pointArray)
            regionPoints = [(pt[0],pt[1]) for pt in pointArray]
            self.currentRegionPoly = Polygon(regionPoints)
            if self.rfi.regions[current_reg].name.lower()=='freespace':
                for region in self.rfi.regions:
                    if region.name.lower() != 'freespace':
                        pointArray = [x for x in region.getPoints()]
                        pointArray = map(self.coordmap_map2lab, pointArray)
                        regionPoints = [(pt[0],pt[1]) for pt in pointArray]
                        self.currentRegionPoly = self.currentRegionPoly - Polygon(regionPoints)
                        
            #set to zero velocity before finding the tranFace
            self.drive_handler.setVelocity(0, 0)
            if last:
                transFace = None
            else:
                print "Current reg is " + str(self.rfi.regions[current_reg].name.lower())
                print "Next reg is "+ str(self.rfi.regions[next_reg].name.lower())
                
                
                for i in range(len(self.rfi.transitions[current_reg][next_reg])):
                    pointArray_transface = [x for x in self.rfi.transitions[current_reg][next_reg][i]]
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
                        pt1  = tf
                        max_magsq = magsq
                transFace = 1   

                self.q_g[0] = pt1[0]
                self.q_g[1] = pt1[1]
                
                
                # Push the goal point to somewhere inside the next region to ensure the robot will go there.
                if self.rfi.regions[current_reg].name.lower()=='freespace':    
                    self.q_g = self.q_g+(self.q_g-asarray(self.nextRegionPoly.center()))/norm(self.q_g-asarray(self.nextRegionPoly.center()))*3*self.PioneerLengthHalf   
                    if not self.nextRegionPoly.isInside(self.q_g[0],self.q_g[1]):  
                        self.q_g = self.q_g-(self.q_g-asarray(self.nextRegionPoly.center()))/norm(self.q_g-asarray(self.nextRegionPoly.center()))*6*self.PioneerLengthHalf 
                else:
                    self.q_g = self.q_g+(self.q_g-asarray(self.currentRegionPoly.center()))/norm(self.q_g-asarray(self.currentRegionPoly.center()))*3*self.PioneerLengthHalf   
                    if self.currentRegionPoly.isInside(self.q_g[0],self.q_g[1]):  
                        self.q_g = self.q_g-(self.q_g-asarray(self.currentRegionPoly.center()))/norm(self.q_g-asarray(self.currentRegionPoly.center()))*6*self.PioneerLengthHalf 
                        
                #plot exiting point  
                if self.PLOT_EXIT == True:
                    plt.figure(self.overlap_figure) 
                    plt.clf()                    
                    plt.plot(q_gBundle[:,0],q_gBundle[:,1],'ko' )   
                    plt.plot(self.q_g[0],self.q_g[1],'ro')
                    plt.plot(pose[0],pose[1],'bo')
                    self.plotPioneer(self.overlap_figure,1)                   
                    
                if transFace is None:
                    print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.rfi.regions[current_reg].name, self.rfi.regions[next_reg].name)
        
      
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
        ##Check whether obsRange overlaps with obstacle or the boundary 
        
        # for real Pioneer robot
        if self.system == 1:
            if self.boundary_following == False:

                if self.robocomm.getReceiveObs() == False:
                    overlap = self.robot & ( self.map_work) 
                else:                   
                    overlap = self.robot  & ( self.map_work | self.robocomm.getObsPoly()) 
                    
            else: #TRUE
                # use a robot with full range all around it
                Robot = Circle(self.obsRange,(pose[0],pose[1]))
                Robot.shift(self.shift*cos(pose[2]),self.shift*sin(pose[2]))            
            
                if self.robocomm.getReceiveObs() == False:
                    overlap = Robot & ( self.map_work) 
                else:
                    
                    overlap = Robot  & ( self.map_work | self.robocomm.getObsPoly()) 
        # for ODE           
        else:
            if self.boundary_following == False:
                overlap = self.robot  & (self.map_work)
            else:#TRUE
                overlap = Robot  & (self.map_work)
        
            
        if self.boundary_following == False:
            if bool(overlap):    ## overlap of obstacles
                #print "There MAYBE overlap~~ check connection to goal"  
                
                # check whether the real robot or and path to goal overlap with the obstacle
                QGoalPoly= Circle(self.PioneerLengthHalf,(self.q_g[0],self.q_g[1])) 
                path  = convexHull(self.realRobot + QGoalPoly)
                if self.system == 1:
                    if self.robocomm.getReceiveObs() == False:
                        pathOverlap = path & ( self.map_work) 
                    else:                
                        pathOverlap = path  & ( self.map_work | self.robocomm.getObsPoly()) 
                else:
                    pathOverlap = path & ( self.map_work) 


                if bool(pathOverlap):   # there is overlapping, go into bounding following mode
                    print "There IS overlap"
                    self.q_hit  = mat([pose[0],pose[1]]).T
                    self.boundary_following = True
                    
                    #Generate m-line polygon
                    QHitPoly = Circle(self.PioneerLengthHalf/4,(pose[0],pose[1]))
                    QGoalPoly= Circle(self.PioneerLengthHalf/4,(self.q_g[0],self.q_g[1]))     
                    self.m_line  = convexHull(QHitPoly + QGoalPoly)                   
                    
                    #plot the first overlap
                    if self.PLOT_M_LINE == True:
                        plt.figure(self.overlap_figure)
                        plt.clf()                        
                        self.plotPoly(QHitPoly,'k')
                        self.plotPoly(QGoalPoly,'k')
                        self.plotPoly(overlap,'g')
                        self.plotPoly(self.m_line,'b')
                        plt.plot(pose[0],pose[1],'bo')
                        self.plotPioneer(self.overlap_figure,1) 
                                        
                  
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
                        print "no obstacles 2-ODE true"
                        
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
                        print "no obstacles 1-ODE true"
        
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
                if x > 0 and y > 0:
                    angle = angle
                elif x < 0 and y > 0:
                    angle = pi + angle
                elif x <0 and y < 0:
                    angle = pi + angle
                else: 
                    angle = 2*pi + angle
                
                
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
                    Robot = Rectangle(self.range*2*j,self.range*2*j)
                    Robot.shift(pose[0]-self.range*j*2,pose[1]-self.range*j)
                    Robot.rotate(pose[2]-pi/2,pose[0],pose[1])
                    
                # on the right
                else: 
                    #print "on the right, angle: "+ str(angle) + " omega: "+ str(omega)+ " angle-omega: "+ str(angle-omega)
                    Robot = Rectangle(self.range*2*j,self.range*2*j)
                    Robot.shift(pose[0],pose[1]-self.range*j)
                    Robot.rotate(pose[2]-pi/2,pose[0],pose[1])
                
                if self.system == 1:
                    overlap = Robot & ( self.map_work | self.robocomm.getObsPoly()) 
                else:
                    overlap = Robot & ( self.map_work) 
                
                self.plotPoly(Robot, 'm',2) 
                
            ##extra box plotting in figure 1#
            if self.PLOT_OVERLAP == True: 
                plt.figure(self.original_figure)
                plt.clf()  
                self.plotPoly(self.realRobot, 'r')
                self.plotPoly(self.robot, 'b') 
                self.plotPoly(overlap,'g',3)                  
                plt.plot(pose[0],pose[1],'bo')
                self.plotPioneer(self.original_figure,0) 
            
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
            pt =  self.closest_pt([pose[0],pose[1]], vstack((q_overlap,asarray(pointList(overlap)))))
            self.prev_follow = pt           
            
            #calculate the vector to follow the obstacle
            normal = mat([pose[0],pose[1]] - pt)
            
            #find the distance from the closest point
            distance = norm(normal)            
            velocity = normal * self.trans_matrix
            vx = (velocity/norm(velocity)/3)[0,0]
            vy = (velocity/norm(velocity)/3)[0,1]

            # push or pull the robot towards the obstacle depending on whether the robot is close or far from the obstacle.    
            turn = pi/4*(distance-0.5*self.obsRange)/(self.obsRange)    ### 5
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
                plt.plot(vx,vy,'ko')
                plt.plot(pt[0],pt[1],'ro')
                plt.plot(pose[0],pose[1],'bo')
                self.plotPioneer(self.overlap_figure,1)
                     
                
            ## conditions that the loop will end
            #for 11111
            RobotPoly = Circle(self.PioneerLengthHalf+0.06,(pose[0],pose[1]))   ####0.05
            arrived  = self.nextRegionPoly.covers(self.realRobot)
            
            #for 33333
            reachMLine= self.m_line.overlaps(RobotPoly)
            
            # 1.reached the next region
            if arrived:
                self.boundary_following = False
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
                print >>sys.__stdout__, "m-line overlaps RoboPoly, m-line" + str(norm(self.q_g-self.q_hit)-2*self.obsRange) + " distance: " + str(norm(self.q_g-mat([pose[0],pose[1]]).T))
                if norm(self.q_g-mat([pose[0],pose[1]]).T) < norm(self.q_g-self.q_hit)-2*self.obsRange:
                    print "m-line overlaps RoboPoly, m-line" + str(norm(self.q_g-self.q_hit)-2*self.obsRange) + " distance: " + str(norm(self.q_g-mat([pose[0],pose[1]]).T))
                    print "leaving boundary following mode"
                    self.boundary_following = False
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
                            print >>sys.__stdout__,"omega<0: "+ str(omega)
                        else: 
                            omega = self.pose_handler.getPose()[2]
                            print >>sys.__stdout__,"omega: "+ str(omega)
                        
                        
                        if omega > angle: 
                            cc = 2*pi - (omega - angle) 
                        else: 
                            cc = angle - omega
                            
                        # angle(goal point orientation) on the left of omega(robot orientation) 
                        #if angle - omega > 0 and angle - omega < pi:
                        if cc < pi:
                            print>>sys.__stdout__, "turn left"
                            vx,vy = self.turnLeft(cc)
                            
                        # on the right
                        else: 
                            print>>sys.__stdout__, "turn right"
                            vx, vy = self.turnRight(2*pi-cc)
                        
                        print>>sys.__stdout__, "omega: "+ str(omega) + " angle: "+ str(angle) + " (omega-angle): " + str(omega-angle)   
                        self.drive_handler.setVelocity(vx,vy, self.pose_handler.getPose()[2])
                        
                        if omega - angle < pi/6 and omega - angle > -pi/6:
                            leaving = True
                        
            #Check whether the robot can leave now (the robot has to be closer to the goal than when it is at q_hit to leave)
            QGoalPoly= Circle(self.PioneerLengthHalf,(self.q_g[0],self.q_g[1])) 
            path  = convexHull(self.realRobot + QGoalPoly)
            if self.system == 1:
                if self.robocomm.getReceiveObs() == False:
                    pathOverlap = path & ( self.map_work) 
                else:                
                    pathOverlap = path  & ( self.map_work | self.robocomm.getObsPoly()) 
            else:
                pathOverlap = path & ( self.map_work) 

            if not bool(pathOverlap):
                print "There is NO MORE obstacles in front for now." 
                
                # check if the robot is closer to the goal compared with q_hit
                if norm(self.q_hit-mat(self.q_g).T) > norm(mat([pose[0],pose[1]]).T-mat(self.q_g).T) :                
                    print "The robot is closer than the leaving point. The robot can leave"
                    self.boundary_following = False
                    self.q_hit_count       = 0
                    dis_cur  = vstack((self.q_g[0],self.q_g[1]))- mat([pose[0],pose[1]]).T
                    vx = (dis_cur/norm(dis_cur)/3)[0,0]
                    vy = (dis_cur/norm(dis_cur)/3)[1,0]
                else:
                    print "not leaving bug algorithm. difference(-farther) =" + str(norm(self.q_hit-mat(self.q_g).T) - norm(mat([pose[0],pose[1]]).T-mat(self.q_g).T))
            

        # Pass this desired velocity on to the drive handler
        # Check if there are obstacles within 0.35m of the robot, if so, stop the robot
        if self.system == 1:
            if self.robocomm.getSTOP() == True:
                vx = 0
                vy = 0
            self.drive_handler.setVelocity(vx,vy, pose[2])
            
        # Set the current region as the previous current region(for checking whether the robot has arrived at the next region)
        self.previous_current_reg = current_reg        
        
        # check whether robot has arrived at the next region 
        RobotPoly = Circle(self.PioneerLengthHalf+0.06,(pose[0],pose[1]))     ###0.05
        departed = not self.currentRegionPoly.covers(RobotPoly)
        arrived  = self.nextRegionPoly.covers(self.realRobot)
        if arrived:
            self.q_hit_count        = 0
            self.boundary_following = False

        if departed and (not arrived) and (time.time()-self.last_warning) > 0.5:
            print "WARNING: Left current region but not in expected destination region"
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
     
        
    
    def plotPioneer(self,number,y = 1):
        """
        Plotting regions and obstacles with matplotlib.pyplot 
        
        number: figure number (see on top)
        y = 0 : plot self.map_work instead of self.map
        """
        
        if not plt.isinteractive():
            plt.ion()       
        plt.hold(True)
        
        
        if y == 0:
            BoundPolyPoints = asarray(pointList(self.map_work))
            plt.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
        else:
            BoundPolyPoints = asarray(pointList(self.map))
            plt.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
        
        if self.system == 1:
            if bool(self.robocomm.getObsPoly()):        
                BoundPolyPoints = asarray(pointList(self.robocomm.getObsPoly()))
                plt.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
        
        plt.xlabel('x')
        plt.ylabel('y')
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
        c = polygon to be plotted with matlabplot
        string = string that specify color
        w      = width of the line plotting 
        """
        BoundPolyPoints = asarray(pointList(c))
        plt.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],string,linewidth=w)   
    
        
        
