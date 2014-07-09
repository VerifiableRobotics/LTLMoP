"""
================================================================================
HexapodDriveHandler.py - The Hexapod's Drive Handler
================================================================================
"""

import math
import logging
import globalConfig

import lib.handlers.handlerTemplates as handlerTemplates

class HexapodDriveHandler(handlerTemplates.DriveHandler):
    def __init__(self, executor, threshold, shared_data):
        """
        Drive handler for Hexapod

        threshold (int): the max angle in degrees difference allowed between the robot head direction and the velocity direction (default=30)
        """

        # get serial port of hexapod
        try:
            self.hexapodSer = shared_data["hexapodSer"]
        except:
            logging.exception("Couldn't connect to Hexapod")
            exit(-1)

        # Get reference to locomotion command handler, since
        # we'll be passing commands right on to it
        self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)

        self.threshold = threshold

    def setVelocity(self, x, y, theta=0):
        """
        Given desired translational [X,Y] velocity specified in the
        *GLOBAL REFERENCE FRAME* from the motion controller, and current robot
        orientation theta (measured counter-clockwise from the positive X-axis),
        construct a drive command for our robot that will get us as close as
        possible and send it on to the locomotion command handler
        """

        if x == 0 and y == 0:
            self.loco.stop()
        else:
            direction = self.getDirection(x, y, theta)
            if self.thresholdIsMet(direction,theta) and direction < 180:
                self.loco.turnClockwise()
            elif self.thresholdIsMet(direction,theta) and direction >= 180:
                self.loco.turnCounterclockwise()
            else:
                self.loco.move()

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

        #angle to go towards relative to hexapod's facing
        direction = angleVelGlob - theta
        #assure it is positive
        while (direction < 0):
            direction += 360

        return direction

    def thresholdIsMet(self, direction, theta):
        """
        Checks to see if the relative angle we wish to go is significantly
        bigger or smaller than the one we are currently heading towards.
        @param direction: The direction we wish to head towards with respect
                          to our facing direction
        @return: True if angle is large enough, False otherwise
        """

        dif = direction - theta
        while  dif > 180:
            dif -= 360
        while  dif < -180:
            dif += 360

        if (math.fabs(dif) < self.threshold):
            return False
        else:
            return True
