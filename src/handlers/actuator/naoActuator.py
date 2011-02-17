#!/usr/bin/env python
"""
===============================================
naoActuator.py - Nao Actuator Handler
===============================================

Control predefined Nao motion besides walking
"""

import time
import naoqi
from naoqi import ALProxy

class actuatorHandler:
    def __init__(self, proj, shared_data):
        # Get proxies
        try:
            self.movProxy = shared_data['mov']
            self.ttsProxy = shared_data['tts']
            self.ledProxy = shared_data['led']
        except KeyError, ValueError:
            print "(ACT) ERROR: No proxy set to correct key in initialization handler for 'mov' 'tts' or 'led'."
            exit(-1)

    def setActuator(self, name, val):
        # Set face to light up or not according to val
        if name == 'FaceLightON':
            if val == 1:
                self.ledProxy.setIntensity('FaceLeds',1)
            else:
                pass
        elif name == 'FaceLightOFF':
            if val == 1:
                self.ledProxy.setIntensity('FaceLeds',0)
            else:
                pass
        elif name == 'Greet':
            if val == '1':
                self.ttsProxy.say("Hello")
                time.sleep(0.5)
            else:
                self.ttsProxy.say("Goodbye")
                time.sleep(0.5)
        elif name == 'LightON':
            # Placeholder to handle custom proposition
            # This should be fixed in future LTLMoP releases
            pass
        else:
            print "(ACT) ERROR: No such actuator name as %s." % name
            pass

        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

if __name__ == '__main__':
    # Test the actuator handler
    naoIP = 'nao.local'
    naoPort = 9559
    movProxy = ALProxy('ALMotion',naoIP,naoPort)
    ttsProxy = ALProxy('ALTextToSpeech',naoIP,naoPort)
    ledProxy = ALProxy('ALLeds',naoIP,naoPort)
    shared_data = {'mov':movProxy,'tts':ttsProxy,'led':ledProxy}
    actuator_handler = actuatorHandler(None, shared_data)
    
    i = 0
    while i < 10:
        actuator_handler.setActuator('FaceLightON','1')
        actuator_handler.setActuator('Greet','1')
        print "on"
        time.sleep(1)
        actuator_handler.setActuator('FaceLightOFF','1')
        actuator_handler.setActuator('Greet','0')
        print "off"
        time.sleep(1)
        i = i+1
