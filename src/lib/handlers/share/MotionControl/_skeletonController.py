#!/usr/bin/env python
"""
==================================================
skeletonController.py - Skeleton Motion Controller

A motionControl handler is the main module for LTLMoP executor to control the locomotion of
the robot. LTLMoP executor will provide  the current robot region and the target robot region
as specified in the synthesized strategy. Then motionControl handler should plan a feasible
path for the robot to safely nevigate to the target region. The motionControl handler should
then calculate global velocity command based on the planed path and the robot current position.
The global velocity is then passed on to the drive handler which should convert the global velocity
vector to the robot local frame velocity.
==================================================
"""

from numpy import *

class motionControlHandler:
    def __init__(self, executor, shared_data):
        """
        Initialize a motionControl handler.

        executor - LTLMoP executor object. See src/lib/executor.py
        shared_data - a dictionary that holds data that are shared among all handlers.
        """
        # Get references to handlers we'll need to communicate with
        self.drive_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler)
        self.pose_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler)

        # Get information about regions
        self.rfi = executor.proj.rfi
        self.coordmap_map2lab = executor.hsub.coordmap_map2lab

    def gotoRegion(self, current_reg, next_reg):
        """
        This function will be called by the LTLMoP executor to control the locomotion of the robot
        based on the synthesized strategy.

        current_reg - index of the region which the robot is currently in.
        next_reg - index of the region which the robot should head to.

        Returns ``True`` if we've reached the destination region.
        """

        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return True  # Already there

        # Find our current configuration
        pose = self.pose_handler.getPose()

        # NOTE: Information about region geometry can be found in self.rfi.regions:
        pointArray = [x for x in self.rfi.regions[current_reg].getPoints()]
        pointArray = map(self.coordmap_map2lab, pointArray)
        vertices = mat(pointArray).T

        # TODO: Calculate a velocity vector in the *GLOBAL REFERENCE FRAME*
        # that will get us on our way to the next region
        vx, vy = 0, 0

        # Pass this desired velocity on to the drive handler
        # It also gets the current heading of the robot
        self.drive_handler.setVelocity(vx, vy, pose[2])

        # Figure out whether we've exited the current region
        if is_inside([pose[0], pose[1]], vertices):
            arrived = False
        else:
            arrived = True

        return arrived
