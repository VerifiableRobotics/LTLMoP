#!/usr/bin/env python
"""
=========================================================
Johnny5SensorHandler.py - Johnny 5 Robot Sensor Handler
=========================================================
"""

import time
import math

import lib.handlers.handlerTemplates as handlerTemplates
import lib.handlers.share.Pose._pyvicon as _pyvicon

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

        self.pose_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler)

    ###################################
    ### Available sensor functions: ###
    ###################################
    def findPoint(self, initial=False):

        if initial:
            print "Connecting to Vicon server..."
            self.viconServer = _pyvicon.ViconStreamer()
            self.viconServer.connect("10.0.0.102", 800)

            model_name = "GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01"
            self.viconServer.selectStreams(["Time"] + ["{} <{}>".format(model_name, s) for s in ("t-X", "t-Y")])
            self.viconServer.startStreams()

            # Wait for first data to come in
            while self.viconServer.getData() is None:
                pass
        else:
            (t, x, y) = self.viconServer.getData()
            (t, x, y) = [t/100, x/1000, y/1000]

            # Find our current configuration
            pose = self.pose_handler.getPose()

            range = 0.7
            # Return true if robot is within range of helmet
            return math.sqrt((pose[0]-x)**2+(pose[1]-y)**2)<range

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

            if hand=='left' and left>=threshold:
                return True
            if hand=='right' and right>=threshold:
                return True

            return False

