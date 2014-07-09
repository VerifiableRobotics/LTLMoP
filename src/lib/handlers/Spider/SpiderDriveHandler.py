#!/usr/bin/env python
"""
=================================================
SpiderDrive.py - DriveHandler for Spider
=================================================
Converts a desired global velocity vector into one of the configured gaits and
angle
"""

import math

import lib.handlers.handlerTemplates as handlerTemplates

class SpiderDriveHandler(handlerTemplates.DriveHandler):
    threshold = 30      #the min angle difference for changing  the gait
    possibleIntervals = 30


    def __init__(self, executor, shared_data):
        self.stopped = True

        try:
            self.spiderSer = shared_data["SpiderSer"]
        except:
            print "(DRIVE) ERROR: No connection to  Spider"
            exit(-1)

        try:
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
            self.coordmap = executor.hsub.coordmap_lab2map
            self.commands = shared_data["Commands"]
        except NameError:
            print "(DRIVE) ERROR: Locomotion Command Handler not found."
            exit(-1)

        self.loco.performGait("standUp")
        self.loco.state = "standing"


    def setVelocity(self, x, y, theta=0):
        """
        Will reference a state machine before deciding what to do, then will
        take in a global x and y velocity vector, as well as the direction
        the robot is facing, and turn it into a relative velocity angle. It
        then decides if the angle is small and should be ignored or if the
        gait and direction of the spider should be changed.
        @param x: global velocity x component
        @param y: global velocity y component
        @param theta: global angle the robot is facing (radians)
        """

        if self.loco.state == "walking":
            if x == 0 and y == 0:
                self.loco.setMoving(False)
            else:
                direction = self.getDirection(x, y, theta)
                if not self.thresholdIsMet(direction):
                    return
                possibleDir = self.getNearestAngle(direction)
                self.loco.setWalkingGait(possibleDir)

        elif self.loco.state == "stopped":
            if x == 0 and y == 0:
                return
            else:
                direction = self.getDirection(x, y, theta)
                if self.thresholdIsMet(direction):
                    possibleDir = self.getNearestAngle(direction)
                    self.loco.setWalkingGait(possibleDir)
                self.loco.setMoving(True)

        elif self.loco.state == "standing":
            if x == 0 and y == 0:
                return
            else:
                direction = self.getDirection(x, y, theta)
                possibleDir = self.getNearestAngle(direction)
                self.loco.setWalkingGait(possibleDir)
                self.loco.setMoving(True)

        elif self.loco.state == "sitting":
            return

        else:
            print "(LOCO) unknown state:", self.loco.state


    def getDirection(self, x, y, theta):
        """
        Take in a global x and y velocity vector, as well as the direction the
        robot is facing, and turn it into a relative heading direction.
        @param x: global velocity x component
        @param y: global velocity y component
        @param theta: global angle the robot is facing (radians)
        @return direction: the direction for the robot to head relative to
                            itself
        """
        theta = math.degrees(theta) * -1    # will be working clockwise
        #if vicon sends any negative angle, make it positive
        while (theta < 0):
            theta += 360

        angleVelGlob = math.degrees(math.atan2(y, x)) * -1
        #assure dirVelGloabal is pos
        while (angleVelGlob < 0):
            angleVelGlob += 360

        #angle to go towards relative to spider's facing
        direction = angleVelGlob - theta
        #assure it is positive
        while (direction < 0):
            direction += 360

        return direction

    def getNearestAngle(self, angle):
        """
        Gets the nearest possible angle to the desired one based on
        possibleIntervals.
        @param angle: The angle to go towards (must be positive)
        @return: The nearest possible angle
        """
        possibleDir = (int(angle + self.possibleIntervals / 2)
                    / self.possibleIntervals * self.possibleIntervals )
        while possibleDir >= 360:
            possibleDir -= 360

        return possibleDir


    def thresholdIsMet(self, direction):
        """
        Checks to see if the relative angle we wish to go is significantly
        bigger or smaller than the one we are currently heading towards.
        @param direction: The direction we wish to head towards with respect
                          to our facing direction
        @return: True if angle is large enough, False otherwise
        """
        if self.loco.currentDir == None:
            return True

        dif = direction - self.loco.currentDir
        while  dif > 180:
            dif -= 360
        while  dif < -180:
            dif += 360

        if (math.fabs(dif) < self.threshold):
            return False
        else:
            return True

