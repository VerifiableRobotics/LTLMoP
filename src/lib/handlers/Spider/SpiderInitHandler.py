#!/usr/bin/env python
"""
================================================================================
SpiderInit.py -- Spider Initialization Handler
================================================================================
Some notes about the spider robot:
    Servo - ranges are about 700 to 2250, but some will hit other parts and will
    not go the full range
            shoulder:    high -> counter clockwise motion
                         low  -> clockwise motion
            elbow:       high -> down
                         low  -> up
            wrist:       high -> in
                         low  -> out
    Legs - The legs will be referred to from 0 - 5 starting with the forward
           right leg and moving clockwise.
           The order of the servo's index will go from shoulder to elbow to
           wrist. Ex) leg0 shoulder is 0, leg0 wrist is 2, leg2 elbow is 7
"""

"""
TODO:
    notes about easy debugging
    format of gait
    limitations with xbee speeds
    gaits and preparation gaits
        limitation with memory and size of gaits
        must be the same on arduino and pc
    range of possible movements
    how each leg is referred to as well as each joint of the leg
    basic states that the spider could have and current direction
        gait indexes
    checksum not always get there ok
"""

import serial, copy, os

import lib.handlers.handlerTemplates as handlerTemplates

class SpiderInitHandler(handlerTemplates.InitHandler):
    #Defaults
    baud = 9600
    timeout = 1         #in seconds

    # These are the commands that can be sent to the Arduino
    commands = {"STOP_MOVING": 0xA0,
                "CONTINUE_MOVING": 0xA1,
                "TURN_OFF_SERVOS": 0xA2,
                "CHANGE_GAIT": 0xA3,
                "PERFORM_GAIT":0xA4,
                "STORE_GAIT": 0xA5,
                "SEND_DIRECTLY": 0xA6,
                "STOP_SENDING_DIRECTLY": 0xA7,
                "GET_BRIGHTNESS": 0xA8}

    gaitIndex = {"standUp": 0,
                 "sitDown": 1,
                 "walk1": 2,
                 "walk1Prepare": 3,
                 "walk2": 4,
                 "walk2Prepare": 5}


    def __init__(self, executor, comPort):
        """
        The initialization for the Spider

        comPort (string): The comport to connect to (default="COM8")
        """
        self.spiderSer = None   #serial port to spider
        self.gaits = {}         #dictionary of gaits indexed by string of names
        self.walkingGaits = ["walk1", "walk2"]

        try:
            self.spiderSer = serial.Serial(port = comPort, baudrate =
                                           self.baud, timeout = self.timeout)
        except:
            print ("(INIT) ERROR: Couldn't connect to Spider")
            exit(-1)

        try:
            # assuming the gaits file is in the same directory of this init handler
            path = os.path.join(os.path.dirname(__file__), "SpiderGaits.txt")
            p = _Parser(path)
            self.gaits = p.gaits
        except:
            print ("(INIT) ERROR: Couldn't find the spider's gait file")
            exit(-1)

    def Stop(self):
        print "(INIT) Shutting down serial port!"
        self.spiderSer.close()

    def getSharedData(self):
        """
        Return a dictionary of any objects that will need to be shared with
        other handlers
        """
        return {"SpiderSer":self.spiderSer, "Gaits":self.gaits,
                "GaitIndex":initHandler.gaitIndex,
                "WalkingGaits":self.walkingGaits, "AllServoOffset":_Gait.allServoOffset,
                "AllServoMapping":_Gait.allServoMapping, "Commands":initHandler.commands}


"""
===================================================
Helper Classes and Methods
===================================================
The following are data structures that will hold the gaits as well as read
pre-made gaits from a file
"""

class _Gait:
    """
    Stores the moves in a 2D array. Each row will represent a move and each
    index of the column will represent the servo to which the value belongs,
    with the exception of the 18th index which will hold the time in mS that
    the move should take. The value at each element will be the position that
    the servo should go to and if the servo's position should remain the same
    then a -1 will symbolize this. Keep in mind that the servos on the spider
    were not mounted perfectly and there for there is an array for calculating
    the right servo position value based on the offset that was written by
    manual calibration. The arduino will do the offset for each servo and
    it will map each servo on to its proper pin but both of the arrays will
    be here if they are needed for anything else
    """

    '''
    The offset for each of the servo's pulse. The index is the int with which
    we refer to the servo and the value is the offset.
    '''
    allServoOffset = [10, 30, -10,      #leg 0
                      60, -50, -150,    #leg 1
                      -30, 0, 0,        #leg 2
                      60, 0, -60,       #leg 3
                      -50, 10, -60,     #leg 4
                      -120, -40, 10]    #leg 5

    '''
    The mapping of each servo. The index is the int with which we refer to the
    servo and the value at each index is the pin on the motor controller.
    '''
    allServoMapping = [12, 13, 14,    #leg 0
                       4, 5, 6,     #leg 1
                       0, 1, 2,     #leg 2
                       16, 17, 18,  #leg 3
                       20, 21, 22,  #leg 4
                       24, 25, 26]  #leg 5

    def __init__(self, name, angle):
        self.name = name
        self.angle = angle
        self.offset = angle
        self.moves = []

    def addMove(self, mov):
        """
        @param mov: an int array of length 19 that will contain all of the servo
                    movements to be added as well as the time for the move to take
        """
        self.moves.append(copy.deepcopy(mov))

    def rotate(self, angle):
        """
        Will rotate the direction of the gait to the  desired angle in a
        clockwise direction. The increment will result only as a multiple of
        60 (following the symetry of the spider)
        @param angle: The amount to rotate by should be between 0 and 360
                          non-inclusive
        """
        if angle < 0 or angle >= 360:
            return

        rotations = self.getRotationTo(angle)#round to nearest 60 degree rotation

        self.angle += 60 * rotations        #update angle
        while self.angle >= 360:            #range 0 to 360
            self.angle -= 360

        for mov in self.moves:
            tempMov = copy.deepcopy(mov)
            for i in range(len(mov) - 1):
                index = i + rotations * 3
                if index > 17:
                    index -= 18

                mov[index] = tempMov[i]

    def getRotationTo(self, angle):
        """
        Will return an integer representing the number of times the gait
        should be rotated to reach the desired angle. The increment will
        result only as a multiple of 60 (following the symetry of the spider)
        @param angle: The amount to rotate by should be [0, 360)
        @return: The number of rotations to make
        """
        if angle < 0:   #angle must be greater than 0
            return 0

        angleDiff = angle - self.angle

        while angleDiff < 0:
            angleDiff += 360

        #round to nearest 60 degree rotation
        rotations = int(angleDiff - self.offset + 30)/60

        while rotations > 5:
            rotations -= 6

        return rotations

    def getPossibleAngles(self):
        """
        Get the possible angles that this gait can walk in
        @return: An array of the possible angles
        """
        angles = [0]*6
        for i in range(6):
            tempAngle = self.offset + 60 * i
            while tempAngle >= 360:
                tempAngle -= 360
            angles[i] = tempAngle

        return angles

    def clone(self):
        """
        @return: A clone of the gait instance (deep copy)
        """
        retGait = _Gait(self.name, self.angle)
        retGait.moves = copy.deepcopy(self.moves)
        return retGait

    def cloneAndRotate(self, angle):
        """
        Will return a clone of the gait that is also rotated to the angle
        angle
        @param angle: The amount to rotate by should be between 0 and 360
                          non-inclusive
        @return: A rotated clone of this gait
        """
        retGait = self.clone()
        retGait.rotate(angle)
        return retGait

    def printGait(self):
        """
        Prints out the gaits in an easy to copy format so that they can be
        transfered to the arduino easily
        """
        print "int", self.name + "[][19] =",
        out =  "{"
        for mov in self.moves:
            out += "{"
            for i in range(19):
                out += str(mov[i])
                out += ", "
            out = out[:-2]
            out += "},\n"
        out = out[:-2] + "};"    # remove the last comma and \n
        print out + "\n"


class _Parser:
    """
    This class will read from a file, obtain all of the pre-defined gaits, and
    return a dictionary of them with the gait name as the key and the gait
    objects as the data. The gait file must adhere to the following format:
       1- Any line starting with '#' will be regarded as a comment and ignored
           unless it is in between the servos, positions, and time. (nothing must
           go in between these 3 lines!)
       2- Every gait must start with "@ <gait name>" and end with "~"
       3- After the gait name the first line should contain only the angle of
           movement, the direction that this gait will cause the spider to move
           towards with respect to its facing direction (bearing). This is used
           for rotating the gait and thus could just be 0 if it will not be
           rotated. This angle should be as close to 0 as possible but not less
           than 0
       4- Within the gait there will be moves made up of three lines each where
           the first line is each servo, the second is the position you wish each
           servo to go to (in the same order as the order of the servos on the
           previous line), and the time in millisecond that you wish for the move
           to take.
       5- Each servo and position should be separated from the others by spaces or
           tabs
    """
    def __init__(self, path):
        self.gaits = {}         #where all of the gaits will be stored
        f = open(path,'r')

        keepReading = True
        while keepReading:
            line = f.readline()

            if len(line) == 0:          #EOF
                keepReading = False
                break

            line = line.strip()
            if len(line) == 0:          #empty line
                continue

            elif line[0] == '@':        #start of a gait
                line = line[1:]         #remove @
                name = line
                line = f.readline().strip()     #get the angle
                angle = int(line)

                tempGait = _Gait(name, angle)
                self.addAllMoves(tempGait, f)

                self.gaits[name] = tempGait

        f.close()
        print "Done parsing"


    def addAllMoves(self, thisGait, f):
        """
        Will parse all of the moves for this gait and add them one by one
        @param thisGait: the gait to which the moves will be added
        @param f: the file that is being read
        """
        while True:                 #read untill the '~' character is reached
            line = f.readline()
            line = line.strip()

            if len(line) == 0:              #empty line
                continue
            elif line[0] == '#':            #comment skip
                continue
            elif line[0] == '~':            #end of gait
                return
            else:                           #it is a valid move
                #get array of servos, positions, and the time
                ser = line.split()
                servos = [int(x) for x in ser]
                pos = f.readline().split()
                positions = [int(x) for x in pos]
                t = f.readline().strip()
                time = int(t)

                #create the move and add it
                newMove = [-1]*19
                for i in range(len(servos)):
                    newMove[servos[i]] = positions[i]
                newMove[18] = time
                thisGait.addMove(newMove)

