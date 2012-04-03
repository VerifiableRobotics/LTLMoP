#!/usr/bin/env python
"""
=======================================================
holonomicDrive.py - Ideal Holonomic Point Drive Handler
=======================================================

Passes velocity requests directly through.
Used for ideal holonomic point robots.
"""

class driveHandler:
    def __init__(self, proj, shared_data):
        try:
            self.loco = proj.loco_handler
            self.coordmap = proj.coordmap_lab2map
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
        #print "VEL:%f,%f" % tuple(self.coordmap([x, y]))
        self.loco.sendCommand([x,y])

