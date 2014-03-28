#!/usr/bin/env python
"""
=================================================
NXTDrive.py - LEGO Mindstorms NXT Drive Handler
=================================================

Converts a desired global velocity vector into translational and rotational rates for a differential-drive robot,
using feedback linearization.
"""

from math import sin, cos, atan2

import lib.handlers.handlerTemplates as handlerTemplates

class NXTDriveHandler(handlerTemplates.DriveHandler):
    def __init__(self, executor, shared_data):
        """
        Initialization method of drive handler for any NXT.
        """   

        try:
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler) 
            self.coordmap = executor.hsub.coordmap_lab2map
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

        #self.d = d
    def setVelocity(self, x, y, theta=0):
        """Defining the velocity to send to the NXT"""        
        vx = x
        vy = y
        
        self.loco.sendCommand([vx,vy])

