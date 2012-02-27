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

class driveHandler:
    def __init__(self, proj, shared_data):
        """
        Initialization method of bipedalDrive handler.
        """   

        try:
            # Get locomotion command handler to be called in setVelocity
            self.loco = proj.loco_handler
            
            # ?? Get pose handler (don't trust motion controller theta value??
            self.pose = proj.pose_handler
            
            # ?? Get map coordinate transformation method for printing ??
            self.coordmap = proj.coordmap_lab2map
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
        #print "VEL:%f,%f" % tuple(self.coordmap([x, y]))
        #print "(drive) velocity = %f,%f" % tuple([x,y]) #???#
        
        # Set constants
        maxspeed = 1      # Scale speed to percentage of Nao maximum
        maxfreq = 1       # Scale step frequency to percentage of Nao maximum
        angcur = pi/3       # If robot is directed between +/- angcur, it walks in a curve (default: pi/3)
        angfwd = pi/12      # If robot is directed between +/- angfwd, it walks straight forward (default: pi/12)
        minvel = 0.3        # If robot is given a velocity less than minvel it doesn't move
                            # Otherwise it turns in place
        
        print >>sys.__stdout__, 180*atan2(y,x)/pi
        # Find direction of where robot should go
        th = numpy.arctan2(y,x)-theta
        while th > pi:
            th = th-2*pi
        while th < -pi:
            th = th+2*pi
        print >>sys.__stdout__, "(drive) th = "+str(th) #??#
        
        # Set velocities based on where robot should go
        f = maxfreq             # Step frequency
        vy = 0                  # Never step sideways
        if numpy.hypot(x,y) < minvel:
            vx = 0              # Don't move
            w = 0
            print >>sys.__stdout__, "(drive) not moving" #??#
        elif numpy.fabs(th) > angcur:
            vx = 0              # Turn in place
            if th > 0:
                w = maxspeed    # Turn left
                print >>sys.__stdout__, "(drive) turning left" #??#
            else:
                w = -maxspeed   # Turn right
                print >>sys.__stdout__, "(drive) turning right" #??#
        elif numpy.fabs(th) > angfwd:
            vx = maxspeed       # Walk forward while turning
            if th > 0:
                w = maxspeed/2  # Turn left
                print >>sys.__stdout__, "(drive) curving left" #??#
            else:
                w = -maxspeed/2 # Turn right
                print >>sys.__stdout__, "(drive) curving right" #??#
        else:
            vx = maxspeed       # Walk straight forward
            w = 0
            print >>sys.__stdout__, "(drive) walking straight" #??#
        
        # Call locomotion handler
        self.loco.sendCommand([vx,vy,w,f])

