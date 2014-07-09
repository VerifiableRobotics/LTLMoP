#!/usr/bin/env python
# -*- coding: cp1252 -*-
"""
================================================================================
SpiderLocomotionCommand.py - The Spider's Locomotion Command Handler
================================================================================
"""

import time

import lib.handlers.handlerTemplates as handlerTemplates

class SpiderLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    """
    This class will tell the spider which gait to do and provide a means of
    communicating with it. In addition, it will hold the current state of the
    spider in case something else needs to stop the process for some time.
    """

    def __init__(self, executor, shared_data):
        self.state = None           # ex. walking, stopped, standing, or sitting
        self.currentDir = None      # angle towards which we are heading

        try:
            self.spiderSer = shared_data["SpiderSer"]
        except:
            print "(LOCO) ERROR: No connection to  Spider"
            exit(-1)

        try:
            self.gaits = shared_data["Gaits"]
            self.walkingGaits = shared_data["WalkingGaits"]
            self.commands = shared_data["Commands"]
            self.gaitIndex = shared_data["GaitIndex"]
            self.allServoMapping = shared_data["AllServoMapping"]
            self.allServoOffset = shared_data["AllServoOffset"]
        except:
            print "(LOCO) ERROR: No shared data"
            exit(-1)


    def performGait(self, gaitName, angle = None):
        """
        Performs a single gait. Raises an exception if it does not receive a
        response from the spider.
        @param gaitName (string): The name of the gait to perform
        @param angle (int): The angle of the gait
        """
        try:
            index = self.gaitIndex[gaitName]
            if angle != None:
                rotation = self.gaits[gaitName].getRotationTo(angle)
            else:
                rotation = 0
        except:
            print "(LOCO)", gaitName, "not found in the dictionary"
            return

        self.sendAndVerify("PERFORM_GAIT")
        self.spiderSer.write(chr(index))
        self.spiderSer.write(chr(rotation))

    def setWalkingGait(self, angle):
        """
        Looks through the available walking gaits and sets up the gait that
        will move towards the input angle
        @param angle: The angle to move towards must be possible given the
                      available gaits
        """
        self.spiderSer.flushInput()
        if angle >= 360:
            angle -= 360

        gaitNotPos = True

        # Find the gait needed for the given angle
        for name in self.walkingGaits:
            possibleAngles = self.gaits[name].getPossibleAngles()
            if angle in possibleAngles:
                gaitNotPos = False
                break

        if gaitNotPos:
            return

        # Special cases of gaits that need preparing (be sure this corresponds
        # to which ever gaits are loaded on to the spider!)
        if ("walk1" in name) or ("walk2" in name):
            prepAngle = angle
            if prepAngle >= 180:
                prepAngle -= 180
            self.performGait(name + "Prepare", prepAngle)

        index = self.gaitIndex[name]
        rot = self.gaits[name].getRotationTo(angle)

        self.sendAndVerify("CHANGE_GAIT")
        self.spiderSer.write(chr(index))
        self.spiderSer.write(chr(rot))

        self.currentDir = angle


    def sendAndVerify(self, cmd, trials = 3, timeout = 5):
        """
        Send a command to the spider and waits untill the spider echoes back
        the command before returning. If all of the trials are carried out and
        there is still no response it will raise an exception.
        @param cmd (string): The key of the command to be sent to the spider
        @param trials: The number of attempts before giving up
        @param timeout: The timeout for each of the trials
        """

        totalTime = trials * timeout + .2
        timeStart = time.clock()

        try:
            command = chr(self.commands[cmd])
        except:
            print "(LOCO)", cmd, "was not found in the command dict."
            return

        self.spiderSer.write(command)
        sentTime = time.clock()

        serialTimeout = self.spiderSer.timeout
        self.spiderSer.timeout = .05

        while (time.clock() - timeStart) < totalTime:
            if (time.clock() - sentTime) > timeout: #resend command
                self.spiderSer.write(command)
                sentTime = time.clock()
            ret = self.spiderSer.read()
            if ret == command:  #got the return
                self.spiderSer.timeout = serialTimeout
                return

        raise Exception("(LOCO) Spider did not respond")

    def storeGait(self, thisGait, gaitIndex):
        """
        Stores a new gait in the location of one of the Spider's prewritten
        gaits. Be sure that the gait you are storing is the same size as the
        one you are overriding
        @param thisGait (Gait): The gait to store
        @param gaitIndex: The index (on the arduino) of the gait to be overwritten
        """
        self.sendAndVerify("STORE_GAIT")
        self.spiderSer.write(chr(gaitIndex))
        self.spiderSer.flushInput()

        for i in range(10):
            checksum = 0
            allMoves = thisGait.moves
            for moves in allMoves:
                for val in moves:
                    # send and calculate checksum
                    byte2 = val & 0xFF
                    byte1 = (val >> 8) & 0xFF
                    checksum += byte1 + byte2
                    checksum = checksum & 0xFF
                    self.spiderSer.write(chr(byte1))
                    self.spiderSer.write(chr(byte2))

            checksum = ((checksum ^ 0xFF) + 1) & 0xFF
            resp = self.spiderSer.read()
            if resp == "" or ord(resp) != checksum:
                print "(LOCO) Warning did not recieve checksum in attempt " + str(i)
                continue;
            else:
                return

        #if all attempts failed then give up
        raise Exception("(LOCO) Did not recieve correnct checksum")


    def streamGait(self, thisGait):
        """
        Causes the Spider to perform a gait by streaming every servo command
        directly to the motor controller as opposed to the Arduino. Good if
        the gait is not used offten and thus not on the Arduino
        @param thisGait (Gait): The gait to be performed
        """
        self.sendAndVerify("SEND_DIRECTLY")
        self.spiderSer.flushInput()
        for mov in thisGait.moves:
            comStr = self.getMoveCommand(mov)
            self.spiderSer.write(comStr)
            time.sleep(mov[18]/1000.0)    #wait for move to be performed

        #check for timed out
        timeout = self.spiderSer.timeout
        self.spiderSer.timeout = 0
        resp = self.spiderSer.read()
        self.spiderSer.timeout = timeout
        if resp != "" and ord(resp) == 0xFF:
            raise Exception("(LOCO) Spider's sendDirectly timed out")

        self.sendAndVerify("STOP_SENDING_DIRECTLY")
        self.currentDir = None

    def getMoveCommand(self, move):
        """
        Gives the string that should be passed to the motor controller so that
        it will perform the move desired
        @param move (int[19]): An array of length 19 containing the move to be performed
        @return: The string that represents the move
        """
        ret = ""                # the string to be returned
        for i in range(18):
            if move[i] == -1:
                continue
            pin = self.allServoMapping[i]
            pos = self.allServoOffset[i] + move[i]
            ret += "#" + str(pin) + "P" + str(pos)

        ret += "T" + str(move[18])   # the time for the move
        ret += "\r"
        return ret


    def turnOffServos(self):
        """
        Sends the signal to cut power to the servos
        """
        self.sendAndVerify("TURN_OFF_SERVOS", 5, 3)

    def setMoving(self, moving=False):
        """
        @param moving (bool): True - set moving, False - stop moving
        """

        if moving:
            self.sendAndVerify("CONTINUE_MOVING", 5, 3)
            self.state = "walking"
        else:
            self.sendAndVerify("STOP_MOVING", 5, 3)
            self.state = "stopped"















