#!/usr/bin/env python
"""
=========================================
dummyActuator.py - Dummy Actuator Handler
=========================================

The actions for the spider. Will be able to change the gaits that the spider will do.
"""
import time


import lib.handlers.handlerTemplates as handlerTemplates

class SpiderSensorHandler(handlerTemplates.SensorHandler):

    def __init__(self, executor, shared_data):
        self.prevIntensity = False

        try:
            self.spiderSer = shared_data["SpiderSer"]
        except:
            print "(SENS) Could not get spider serial port"
            exit(-1)

        try:
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
        except:
            print "(SENS) Could not get the loco"
            exit(-1)

    def itIsDark(self, thresh, initial=False):
        """
        Uses the photoresistor on the Spider to sense the brightness and detect objects
        above it.

        thresh (int): The threshold to consider dark (default=120)
        """
        if initial:
            return False

        try:
            self.loco.sendAndVerify("GET_BRIGHTNESS", 1, .1)
        except:
            print "(SENS) Warning spider did not respond in time"
            return self.prevIntensity

        timeout = self.spiderSer.timeout
        self.spiderSer.timeout = .1
        intensity = self.spiderSer.read()
        self.spiderSer.timeout = timeout

        if intensity == "":
            print "(SENS) Warning did not receive sensor from Spider"
            return self.prevIntensity

        if ord(intensity) < thresh:
            self.prevIntensity = True
            return True
        else:
            self.prevIntensity = False
            return False

