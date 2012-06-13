#!/usr/bin/env python
"""
=================================================
NXTDrive.py - LEGO Mindstorms NXT Drive Handler
=================================================

Converts a desired global velocity vector into translational and rotational rates for a differential-drive robot,
using feedback linearization.
"""

from math import sin, cos, atan2

class driveHandler:
    def __init__(self, proj, shared_data):
        """
        Initialization method of drive handler for any NXT.
        """   

        try:
            self.loco = proj.h_instance['locomotionCommand']
            self.coordmap = proj.coordmap_lab2map
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

        #self.d = d
    def setVelocity(self, x, y, theta=0):
        #Defining the velocity to send to the NXT
        c = .29 #scaling costant / don't run into things constant
        vx = c*x
        vy = c*y
        #The following was used for differentila drive and is confusing
        #w = (1/self.d)*(-sin(theta)*vx + cos(theta)*vy)
        #Essentially speed
        #v = cos(theta)*vx + sin(theta)*vy
        #get wanted direction vector
        #phi = 180*atan2(vy/vx)/pi
        #send speed and direction
        self.loco.sendCommand([vx,vy])

