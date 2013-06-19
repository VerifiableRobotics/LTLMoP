#!/usr/bin/env python
"""
================================================
CKBotSimDrive.py - Simulated CKBot Drive Handler
================================================
"""
import math
from simulator.ode.ckbot import CKBotSimHelper

import lib.handlers.handlerTemplates as handlerTemplates

class CKBotSimDriveHandler(handlerTemplates.DriveHandler):

    def __init__(self, proj, shared_data,gait):
        """
        Drive handler for simulated CKBot robot
        
        gait (int): Initial gait for the robot. For details, refer to the handler file (default=0)
        """
    
        # Get reference to locomotion command handler, since
        # we'll be passing commands right on to it
    
        self.gait = gait
        self.simulator = shared_data['Simulator']
        self.config = shared_data['Config']
        self.pose_handler = proj.pose_handler
        try:
            self.loco = proj.loco_handler
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
    
        # TODO: Given x and y velocities figure out which gait should be selected. For the case of the Snake robot, for example, we choose between left, right and forward gaits here.

        # First get the current pose of the robot.
        pose = self.pose_handler.getPose()
        robotangle = pose[2]
        velangle = math.atan2(y,x)
        rel_heading = velangle-robotangle
        if (rel_heading > math.pi):
            rel_heading = rel_heading - 2*math.pi
        elif (rel_heading < -math.pi):
            rel_heading = rel_heading + 2*math.pi
        old_gait = self.gait

        # Check for zero speed commands to assign no gait.
        #if (x==0 and y==0) and self.simulator.config!="Plus3":
        if self.simulator.config == "Plus3":
            gait = 0            # Do not move
            #print "we aint here"

        else:    

            # If there is a non-zero speed command, select among forwards, left or right turn gaits.
            if self.simulator.config=="Hexapod":
                # If you were originally going straight, keep going straight until you deviate by a larger angle.
                if old_gait == 1:
                    if (rel_heading<math.pi/6 and rel_heading>-math.pi/6):    
                        gait = 1            # Go Straight
                    elif (rel_heading>math.pi-math.pi/6 or rel_heading<-math.pi+math.pi/6):
                        gait = 4            # Back Up
                    elif (rel_heading<-math.pi/6 and rel_heading>-math.pi/2) or (rel_heading > math.pi/2 and rel_heading <math.pi-math.pi/6):
                        gait = 2            # Go Right
                    elif (rel_heading>math.pi/6 and rel_heading<math.pi/2) or (rel_heading<-math.pi/2 and rel_heading>-math.pi+math.pi/6):
                        gait = 3        # Go Left
                # Otherwise, if you were not originally going straight, only select to go straight if within a smaller angle.                
                else: 
                    if (rel_heading<math.pi/24 and rel_heading>-math.pi/24):    
                        gait = 1            # Go Straight
                    elif (rel_heading>math.pi-math.pi/24 or rel_heading<-math.pi+math.pi/24):
                        gait = 4            # Back Up
                    elif (rel_heading<-math.pi/24 and rel_heading>-math.pi/2) or (rel_heading > math.pi/2 and rel_heading <math.pi-math.pi/24):
                        gait = 2            # Go Right
                    elif (rel_heading>math.pi/24 and rel_heading<math.pi/2) or (rel_heading<-math.pi/2 and rel_heading>-math.pi+math.pi/24):
                        gait = 3


            # For the 4-Module Snake or 25 module Hexapod
            # If there is a non-zero speed command, select among forwards, left or right turn gaits.
            elif self.simulator.config=="Snake":
                if (rel_heading<math.pi/24 and rel_heading>-math.pi/24):    
                    gait = 1            # Go Straight
                elif (rel_heading<-math.pi/24):
                    gait = 2            # Go Right
                elif (rel_heading>math.pi/24):
                    gait = 3            # Go Left
                

            elif self.simulator.config=="Tripod":
                if old_gait == 1:
                    if (rel_heading<math.pi/6 and rel_heading>-math.pi/6):    
                        gait = 1            # Go Straight
                    elif (rel_heading<-math.pi/6):
                        gait = 3            # Go Right
                    elif (rel_heading>math.pi/6):
                        gait = 2            # Go Left
                else:
                    if (rel_heading<math.pi/24 and rel_heading>-math.pi/24):    
                        gait = 1            # Go Straight
                    elif (rel_heading<-math.pi/24):
                        gait = 3            # Go Right
                    elif (rel_heading>math.pi/24):
                        gait = 2            # Go Left

                    
            elif self.simulator.config=="Plus":
                if (rel_heading >= -math.pi/4 and rel_heading < math.pi/4):
                    gait = 3             #  Horizontal forward
                elif (rel_heading >= math.pi/4 and rel_heading < 3*math.pi/4):
                    gait = 2            #  Vertical backward
                elif (rel_heading >= 3*math.pi/4 and rel_heading < -3*math.pi/4):
                    gait = 4            #  Horizontal backward
                else:
                    gait = 1            #  Vertical forward    

            elif self.simulator.config=="Plus3":
                gait = 5            # play dead

            elif self.simulator.config=="Loop10":
                gait = 1            # somersault forward

            elif self.simulator.config=="FoldOver":
                gait = 2

            elif self.simulator.config=="Slinky":
                gait = 2            # move forward

            elif self.simulator.config=="Splits":
                gait = 1            # do the splits over and over

            elif self.simulator.config=="Twist":
                gait = 1

            elif self.simulator.config=="Quadriped":
                gait = 1

            else:
                gait = 0

        # If there is gait switching, print a message.
        self.gait = gait
        #if (old_gait != gait):
            #print "Activating Gait No." + str(gait)

        self.loco.sendCommand(gait)

