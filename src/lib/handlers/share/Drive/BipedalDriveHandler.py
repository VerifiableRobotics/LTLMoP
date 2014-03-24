#!/usr/bin/env python
"""
=========================================
bipedalDrive.py - Nao Walk Drive Handler
=========================================

Converts a desired global velocity vector into a desired velocity vector local to the Nao.
"""

from math import sin, cos, sqrt, atan2, pi, fabs
import sys
import numpy
#import naoqi
#from naoqi import ALProxy

import lib.handlers.handlerTemplates as handlerTemplates

class BipedalDriveHandler(handlerTemplates.DriveHandler):
    def __init__(self, executor, shared_data,maxspeed,maxfreq,angcur,angfwd,minvel,silent):
        """
        Drive handler for bipedal robot

        maxspeed (float): Scale speed with repect to Nao maximum speed (default=1.0,min=0.5,max=1.0)
        maxfreq (float): Scale step frequency with repect to Nao maximum frequency (default=1.0,min=0.5,max=1.0)
        angcur (float): If robot is directed between +/- angcur, it walks in a curve (default=1.047,min=0.5236,max=1.571)
        angfwd (float): If robot is directed between +/- angfwd, it walks straight forward (default=0.262,min=0.262,max=0.5236)
        minvel (float): If robot is given a velocity less than minvel it doesn't move. Otherwise it turns in place (default=0.3,min=0.2,max=0.4)
        silent (bool): If true, no debug message will be printed (default=True)
        """

        # Set constants
        self.maxspeed = maxspeed
        self.maxfreq = maxfreq
        self.angcur = angcur
        self.angfwd = angfwd
        self.minvel = minvel
        self.silent = silent

        try:
            # Get locomotion command handler to be called in setVelocity
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
        except NameError:
            if not self.silent: print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)
        try:
            # ?? Get pose handler (don't trust motion controller theta value??
            self.pose = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler)
        except NameError:
            if not self.silent: print "(DRIVE) Pose Handler not found."
            exit(-1)
        try:
            # ?? Get map coordinate transformation method for printing ??
            self.coordmap = executor.hsub.coordmap_lab2map
        except NameError:
            if not self.silent: print "(DRIVE) Calibration data not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
        #if not self.silent: print "VEL:%f,%f" % tuple(self.coordmap([x, y]))
        #if not self.silent: print "(drive) velocity = %f,%f" % tuple([x,y]) #???#


        if not self.silent: print >>sys.__stdout__, 180*atan2(y,x)/pi
        # Find direction of where robot should go
        th = numpy.arctan2(y,x)-theta
        while th > pi:
            th = th-2*pi
        while th < -pi:
            th = th+2*pi
        if not self.silent: print >>sys.__stdout__, "(drive) th = "+str(th) #??#

        # Set velocities based on where robot should go
        f = self.maxfreq             # Step frequency
        vy = 0                  # Never step sideways
        if numpy.hypot(x,y) < self.minvel:
            vx = 0              # Don't move
            w = 0
            if not self.silent: print >>sys.__stdout__, "(drive) not moving" #??#
        elif numpy.fabs(th) > self.angcur:
            vx = 0              # Turn in place
            if th > 0:
                w = self.maxspeed    # Turn left
                if not self.silent: print >>sys.__stdout__, "(drive) turning left" #??#
            else:
                w = -self.maxspeed   # Turn right
                if not self.silent: print >>sys.__stdout__, "(drive) turning right" #??#
        elif numpy.fabs(th) > self.angfwd:
            vx = self.maxspeed       # Walk forward while turning
            if th > 0:
                w = self.maxspeed/2  # Turn left
                if not self.silent: print >>sys.__stdout__, "(drive) curving left" #??#
            else:
                w = -self.maxspeed/2 # Turn right
                if not self.silent: print >>sys.__stdout__, "(drive) curving right" #??#
        else:
            vx = self.maxspeed       # Walk straight forward
            w = 0
            if not self.silent: print >>sys.__stdout__, "(drive) walking straight" #??#

        # Call locomotion handler
        self.loco.sendCommand([vx,vy,w,f])

