#!/usr/bin/env python
"""
================================================================================
Johnny5InitHandler.py -- Johnny 5 Initialization Handler
================================================================================
Some notes about Johnny 5:
    Servo - 0   base rotates
            1   lower waist leans forwards/backwards
            2   upper waist leans forwards/backwards
            3   left shoulder rotates
            4   left shoulder moves inwards/outwards
            5   left arm rotates
            6   left arm moves inwards/outwards
            7   left hand grasps
            8   right shoulder rotates
            9   right shoulder moves inwards/outwards
            10  right arm rotates
            11  right arm moves inwards/outwards
            12  left hand grasps
            13  head rotates
            14  track moves forwards/backwards  static @ 1500; forward @ <1500; backwards @ >1500;
            15  track rotates   static @ 1500; CW @ >1500; CCW @ <1500;

    Servo commands format: #'Servo Num' + P'Servo Val' + T'Time in ms' + \r
"""

import sys
import os
import serial
import logging
import globalConfig

import lib.handlers.handlerTemplates as handlerTemplates

class Johnny5InitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor, comPort):
        """
        The initialization for Johnny 5

        comPort (string): The comport to connect to (default='/dev/tty.usbserial-A600eIiI')
        """
        self.johnny5Serial = None   # serial port to Johnny 5

        #Defaults
        self.baud = 115200
        self.timeout = 1         #in seconds

        try:
            self.johnny5Serial = serial.Serial(port = comPort, baudrate =
                                           self.baud, timeout = self.timeout)
        except:
            logging.exception("Couldn't connect to Johnny 5")
            sys.exit(-1)

        # save .cfg file in folder ConfigFile under Johnny5 folder
        self.config = self.initConfig(os.path.join(os.path.dirname(__file__),'ConfigFile','ConfigSSC32.cfg'))
        self.setDefaultPosition()

    def _stop(self):
        logging.info("Shutting down serial port!")
        self.johnny5Serial.close()

    def initConfig(self, cfgFileName):
        """
            Reads in default configuration file and saves.

            8 lines of data for each servo, 16 servos for Johnny 5,
            In each data set of a servo, only lines 4-8 are of interests:

            4: servo value at neutral position
            5: Min servo value
            6: Max servo value
            7: Min servo degree
            8: Max servo degree

            Generate a 2D array "config" in the following format:

            index:          0          1         2         3          4
            Servo #0 Neutral_ servo Min_servo Max_servo Min_degree Max_degree
            Servo #1       ...
            .
            .
            .
        """
        cfg = [data.strip('\r\n') for data in open(cfgFileName)]
        # config is a 16x5 array, initialized with all 0
        config = [[0 for i in range(5)] for j in range(16)]
        for servoNum in range(16):
            config[servoNum] = map(int, cfg[8*servoNum+3:8*servoNum+8])
        return config

    def setDefaultPosition(self):
        """
        Set Johnny 5 servos to default positions
        """
        time = 1000
        # for servo# 0-15
        for i in range(16):
            self.johnny5Serial.write('#%d P%d T%d\r' % (i, int(self.config[i][0]), time))

    def getSharedData(self):
        """
        Return a dictionary of any objects that will need to be shared with
        other handlers
        """
        return {"Johnny5Serial":self.johnny5Serial,
                "DefaultConfig":self.config}
