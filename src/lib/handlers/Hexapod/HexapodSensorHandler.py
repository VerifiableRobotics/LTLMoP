"""
================================================================================
HexapodSensorHandler.py - The Hexapod's Sensor Handler
================================================================================
"""

import time
import logging
import globalConfig

import lib.handlers.handlerTemplates as handlerTemplates

class HexapodSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        """
        Sensor handler for hexapod
        """

        # get serial port of hexapod
        try:
            self.hexapodSer = shared_data["hexapodSer"]
        except:
            logging.exception("Couldn't connect to Hexapod")
            exit(-1)

    def _sendCommand(self, cmd):
        """
        Send locomotion command ``cmd`` to the robot
        """
        self.hexapodSer.write(cmd)
        time.sleep(0.1)
        byte0 = '\xFF'
        byte1 = byte0
        while not (byte0 == '\x22' and byte1 == '\x23'):        #checks to see if two consecutive packets retrieved are correct
            byte0 = byte1                                        #if true, then read in analog value from force sensor
            byte1 = self.hexapodSer.read()
            if not byte1:                                        #if false, then sending has failed and keep looping
                logging.debug('failed')
                return -1
        reading = self.hexapodSer.read(2)                    #read analog value from force sensors
        return reading

    def _checkLeft(self):
        """
        check left force sensor
        """

        orientation = False
        while not orientation:
            y = self._sendCommand('k')            #send command to Arduino to make reading from force sensor
            if y == -1:                            #if no reading, continue to try reading
                continue
            if y[0] == 'l':                        #if char 'l' is retrieved, next byte retrieved is reading from left force sensor
                orientation = True
                leftSensor = y[1]

        return ord(leftSensor)                    #return integer value of left sensor

    def _checkRight(self):
        """
        check right force sensor
        """

        orientation = False
        while not orientation:
            y = self._sendCommand('l')            #send command to Arduino to make reading from force sensor
            if y == -1:                            #if no reading, continue to try reading
                continue
            if y[0] == 'r':                        #if char 'r' is retrieved, next byte retrieved is reading from right force sensor
                orientation = True
                rightSensor = y[1]

        return ord(rightSensor)                                #return integer value of right sensor

    def isObjectDetect(self, thresh, initial = False):
        """
        Uses the force sensor on the hexapod to detect objects in gripper

        thresh (int): The threshold for amount of force (default=50)
        """
        if initial:
            return False
        else:
            # read in values from force sensor and compare to assigned threshold
            # if any side of the sensor reads value larger then the threshold
            # we consider there is an item in the gripper
            if (self._checkRight() > thresh) or (self._checkLeft() > thresh):
                return True
            else:
                return False
