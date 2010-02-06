#!/usr/bin/env python
"""
Passes velocity requests directly through.
Used for ideal holonomic point robots.
"""

class driveHandler:
    def __init__(self, shared_data, loco_handler):
        self.loco = loco_handler

    def setVelocity(self, x, y):
        self.loco.sendCommand([x,y])

