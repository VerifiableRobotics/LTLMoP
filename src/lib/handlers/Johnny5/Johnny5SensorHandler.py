#!/usr/bin/env python
"""
=========================================================
Johnny5SensorHandler.py - Johnny 5 Robot Sensor Handler
=========================================================
"""

import time
import math
import logging
import globalConfig

import lib.handlers.handlerTemplates as handlerTemplates

class Johnny5SensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        """
        Johnny 5 Robot Sensor handler
        """

        # find johnny5 serial port
        try:
            self.johnny5Serial = shared_data["Johnny5Serial"]
        except:
            logging.exception("No connection to Johnny 5")
            sys.exit(-1)

    ###################################
    ### Available sensor functions: ###
    ###################################

    def itemInHand(self, hand, threshold, initial=False):
        """
        Use force sensors on Johnny 5's hands to detect whether an item is in hand

        hand (string): The hand to detect, left or right
        threshold (int): Minimum acceptable detection confidence (default=100,min=0,max=255)
        """
        if initial:
            pass
        else:
            # read force sensors on both hands
            # VA is analog input port corresponding to right hand force sensor
            # VC is analog input port corresponding to left hand force sensor

            sensorData = []
            # we are expecting 2bytes of data
            while len(sensorData)<2:
                self.johnny5Serial.write('VA VC\r')
                time.sleep(0.2)
                # read 2 bytes
                sensorData = self.johnny5Serial.read(2)

            # convert sensor readings to integer between 0-255
            sensorData = sensorData.encode('hex')
            # extract sensor value for right/left hand
            right = sensorData[0]+sensorData[1]
            right = int(right,16)
            left = sensorData[2]+sensorData[3]
            left = int(left,16)

            if hand not in ['left', 'right']:
                raise ValueError('Cannot recognize hand with value {!r}'.format(hand))
            if hand=='left' and left>=threshold:
                return True
            if hand=='right' and right>=threshold:
                return True

            return False

