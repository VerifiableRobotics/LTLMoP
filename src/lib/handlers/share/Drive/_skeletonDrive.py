#!/usr/bin/env python
"""
=========================================
skeletonDrive.py - Skeleton Drive Handler
=========================================
"""

class driveHandler:
    def __init__(self, executor, shared_data):
        # Get reference to locomotion command handler, since
        # we'll be passing commands right on to it

        try:
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
        # TODO: Given desired translational [X,Y] velocity specified in the
        # *GLOBAL REFERENCE FRAME* from the motion controller, and current robot
        # orientation theta (measured counter-clockwise from the positive X-axis),
        # construct a drive command for our robot that will get us as close as
        # possible and send it on to the locomotion command handler

        cmd = [x,y]

        self.loco.sendCommand(cmd)

