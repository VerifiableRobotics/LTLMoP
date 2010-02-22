#!/usr/bin/env python
"""
=========================================
skeletonDrive.py - Skeleton Drive Handler
=========================================
"""

class driveHandler:
    def __init__(self, proj, shared_data):
        # Get reference to locomotion command handler, since
        # we'll be passing commands right on to it

        try:
            self.loco = proj.loco_handler
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, w=0):
        # TODO: Given desired translational [X,Y] and rotational [W] velocities
        # specified in the *GLOBAL REFERENCE FRAME* from the motion controller,
        # construct a drive command for our robot that will get us as close as
        # possible and send it on to the locomotion command handler
        
        cmd = [x,y]
    
        self.loco.sendCommand(cmd)

