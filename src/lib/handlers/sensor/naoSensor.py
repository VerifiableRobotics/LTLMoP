#!/usr/bin/env python
"""
===========================================
naoSensor.py - Nao Sensor Handler
===========================================

Read any of the many Nao sensors
"""

import naoqi
from naoqi import ALProxy
import time

class sensorHandler:
    def __init__(self, proj, shared_data):
        # Get proxy
        try:
            self.memProxy = shared_data['mem']
        except KeyError, ValueError:
            print "(SENSOR) ERROR: No ALMemory proxy set to key 'mem' in initialization handler."
            exit(-1)
        
        # Define dictionary of valid sensor names
        # Keys are the sensor names known to LTLMoP
        # Values are from the data list to call the memory proxy with
        self.sensors = {'TouchFront':'FrontTactilTouched','TouchMiddle':'MiddleTactilTouched',
            'TouchRear':'RearTactilTouched','BumpLeft':'LeftBumperPressed',
            'BumpRight':'RightBumperPressed','ChestButton':'ChestButtonPressed'}
            # Need to add sonar, face detection, logo detection, voice, foot contact, 
            # position? (sitting, lying), motion? (walking, transitioning)

    def getSensorValue(self, sensor_name):
        """Read the state of the sensor with name sensor_name
            Returns the latest reading stored in memory
            If sensor_name is not valid, returns None
            
            sensor_name - string, one of the keys defined in dictionary self.sensors above"""
        
        # Extract sensor reading from memory
        if sensor_name in self.sensors:
            state = self.memProxy.getData(self.sensors[sensor_name],0)
            state = bool(state)
            return state
        else:
            return None

if __name__ == '__main__':
    # Test the sensor handler
    naoIP = 'nao.local'
    naoPort = 9559
    memProxy = ALProxy('ALMemory',naoIP,naoPort)
    shared_data = {'mem':memProxy}
    sensor_handler = sensorHandler(None, shared_data)
    
    i = 0
    while i < 15:
        val = sensor_handler.getSensorValue('TouchFront')
        print val
        time.sleep(1)
        i = i+1
