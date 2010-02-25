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
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
        self.loco.sendCommand([x,y])

