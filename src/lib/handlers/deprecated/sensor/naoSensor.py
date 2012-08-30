#!/usr/bin/env python
"""
===========================================
naoSensor.py - Nao Sensor Handler
===========================================

Read any of the many Nao sensors
"""

import os
import naoqi
from naoqi import ALProxy
import time

class sensorHandler:
    def __init__(self, proj, shared_data):
        # Get proxy
        try:
            self.memProxy = shared_data['mem']
            self.sttProxy = shared_data['stt']
            self.faceProxy = shared_data['face']
        except KeyError, ValueError:
            print "(SENSOR) ERROR: Required handlers not passed from initialization handler."
            exit(-1)
        
        ### Initialize speech-to-text
        subs = [x[0] for x in self.sttProxy.getSubscribersInfo()]
        # Close any previous subscriptions that might have been hanging open
        if "ltlmop_sensorhandler" in subs:
            self.sttProxy.unsubscribe("ltlmop_sensorhandler")
        # Reset the SR register manually
        self.memProxy.insertData("WordRecognized", [])
        self.sttProxy.setLanguage("English")
        self.sttProxy.setAudioExpression(False)
        self.sttProxy.setVisualExpression(True)
        self.sttProxy.setWordListAsVocabulary(["one", "two", "three", "ready"])
        self.sttProxy.subscribe("ltlmop_sensorhandler")
        self.sttNames = self.sttProxy.getOutputNames()
        
        ### Initialize face tracking
        subs = [x[0] for x in self.faceProxy.getSubscribersInfo()]
        # Close any previous subscriptions that might have been hanging open
        if "ltlmop_sensorhandler" in subs:
            self.faceProxy.unsubscribe("ltlmop_sensorhandler")
        self.faceProxy.subscribe("ltlmop_sensorhandler", 100, 0.0)

        # Define dictionary of valid sensor names
        # Keys are the sensor names known to LTLMoP
        # Values are from the data list to call the memory proxy with
        self.sensorMapping = {'touched':'FrontTactilTouched'}
            #,'':'MiddleTactilTouched',
            #'TouchRear':'RearTactilTouched','BumpLeft':'LeftBumperPressed',
            #'BumpRight':'RightBumperPressed','ChestButton':'ChestButtonPressed'}
        
        self.sensorCache = {}
        self.last_update = 0
        self.last_face_val = False

    def getSensorValue(self, sensor_name):
        """Read the state of the sensor with name sensor_name
            Returns the latest reading stored in memory
            If sensor_name is not valid, returns None
            
            sensor_name - string, one of the keys defined in dictionary self.sensors above"""
        
        now = time.time()
        if (now - self.last_update) > 0.1: # Limit update frequency to 10Hz
            ### Request data from Nao
            # Check speech recognition state
            words = self.memProxy.getData("WordRecognized",0)
            self.sensorCache["hear_counting"] = False
            self.sensorCache["hear_whistle"] = False
            for word, prob in zip(words[0::2],words[1::2]):
                if word in ["one", "two", "three"] and prob > 0.2:
                    print "(SENS) Recognized word '%s' with p = %f" % (word, prob)
                    self.sensorCache["hear_counting"] = True
                if word in ["ready"] and prob > 0.2:
                    print "(SENS) Recognized word '%s' with p = %f" % (word, prob)
                    self.sensorCache["hear_whistle"] = True

            # HACK: reset the SR register manually
            self.memProxy.insertData("WordRecognized", [])

            # Check face detection state
            face_data = self.memProxy.getData("FaceDetected",0)
            self.sensorCache["see_player"] = (face_data != [])
            if self.sensorCache["see_player"] and not self.last_face_val:
                print "(SENS) I've seen a new face!"
            self.last_face_val = self.sensorCache["see_player"]

            # Check binary sensor state
            for pname, sname in self.sensorMapping.iteritems():
                state = self.memProxy.getData(sname,0)
                self.sensorCache[pname] = bool(state)
            
            # HACK: for difficult speech cases, use touch too
            if self.sensorCache["touched"]:
                print "(SENS) Touch override"
                #self.sensorCache["hear_counting"] = True
                #self.sensorCache["hear_whistle"] = True
                self.sensorCache["see_player"] = True

            self.last_update = time.time()

        if sensor_name in self.sensorCache:
            return self.sensorCache[sensor_name]
        else:
            print "(SENS) WARNING: Sensor %s is unknown!" % sensor_name
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
