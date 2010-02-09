#!/usr/bin/env python
"""
Converts a desired global velocity vector into translational and rotational rates for a differential-drive robot,
using feedback linearization.
"""

from math import sin, cos

class driveHandler:
    def __init__(self, shared_data, loco_handler):
        self.loco = loco_handler

    def setVelocity(self,x,y,theta=0):
        # Feedback linearization code:
        #d = 0.125 # Distance from front axle to point we are abstracting to [m]
        d = 0.4 # Distance from front axle to point we are abstracting to [m]
        #vx = 0.09*X[0,0]
        #vy = 0.09*X[1,0]
        # ^^ Changed the scaling because it was getting stuck - too high of a velocity ? - Hadas 20/12/07
        vx = 0.29*x
        vy = 0.29*y
        w = (1/d)*(-sin(theta)*vx + cos(theta)*vy)
        v = cos(theta)*vx + sin(theta)*vy

        self.loco.sendCommand([v,w])

