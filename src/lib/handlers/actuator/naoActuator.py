#!/usr/bin/env python
"""
===============================================
naoActuator.py - Nao Actuator Handler
===============================================

Control predefined Nao motion besides walking
"""

import time, os
import naoqi
import threading
from naoqi import ALProxy

class actuatorHandler:
    def __init__(self, proj, shared_data):
        # Get proxies
        try:
            self.movProxy = shared_data['mov']
            self.ttsProxy = shared_data['tts']
            self.ledProxy = shared_data['led']
            self.audProxy = shared_data['aud']
            self.behaviorProxy = shared_data['behavior']
        except KeyError, ValueError:
            print "(ACT) ERROR: No proxy set to correct key in initialization handler for 'mov' 'tts' or 'led'."
            exit(-1)

        print "(ACT) Loading whistle MP3..."
        self.whistleId = self.audProxy.loadFile("/home/nao/data/whistle.mp3")
        #print "(ACT) Preloading Nao behaviors..."
        #self.behaviorProxy.preloadBehavior("sitdown")
        #self.behaviorProxy.preloadBehavior("standup")

        self.doCount = False
        countThread = threading.Thread(target = self.count)
        countThread.start()

    def count(self):
        number = 1
        while 1:
            if self.doCount:
                self.ttsProxy.say(str(number))
                number = number + 1
                time.sleep(10)
            else:
                # Reset the count
                number = 1
                time.sleep(0.1)


    def setActuator(self, name, val):
        # Set face to light up or not according to val
        #if name == 'FaceLightON':
        #    if val == 1:
        #        self.ledProxy.setIntensity('FaceLeds',1)
        #    else:
        #        pass
        #elif name == 'FaceLightOFF':
        #    if val == 1:
        #        self.ledProxy.setIntensity('FaceLeds',0)
        #    else:
        #        pass
        #elif name == 'Greet':
        #    if val == '1':
        #        self.ttsProxy.say("Hello")
        #        time.sleep(0.5)
        #    else:
        #        self.ttsProxy.say("Goodbye")
        #        time.sleep(0.5)
        #elif name == 'LightON':
        #    # Placeholder to handle custom proposition
        #    # This should be fixed in future LTLMoP releases
        #    pass
        if name.lower() == "count":
            if int(val) == 1:
                self.doCount = True
            else:
                self.doCount = False
        elif name.lower() == "say_foundyou":
            if int(val) == 1:
                self.ttsProxy.say("I found you! Let's play again.")
        elif name.lower() == "say_imfound":
            if int(val) == 1:
                self.ttsProxy.say("You got me! Let's play again.")
        elif name.lower() == "say_hider":
            if int(val) == 1:
                self.ttsProxy.say("I'm gonna go hide.  Start counting.")
        elif name.lower() == "say_seeker":
            if int(val) == 1:
                self.ttsProxy.say("I am the seeker.  Go hide and say ready.")
        elif name.lower() == "whistle":
            if int(val) == 1:
                self.audProxy.play(self.whistleId)
                time.sleep(1)
        elif name.lower() == "hide":
            if int(val) == 1:
                self.killBehaviors()
                self.behaviorProxy.runBehavior("sitdown")
                #self.killBehaviors()
                #WARNING: FOR SOME REASON THE FOLLOW CRASHES NAOQI
                #self.movProxy.setAngles(['HeadPitch'], [0.4], 0.2)
            else:
                self.killBehaviors()
                self.behaviorProxy.runBehavior("standup")
                #self.killBehaviors()
                #self.movProxy.setAngles(['HeadPitch'], [0], 0.2)
        elif name.lower() == "playing" or name.lower() == "seeker":
            pass
        else:
            print "(ACT) ERROR: No such actuator name as %s." % name
            pass

        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

    def killBehaviors(self):
        bhv = self.behaviorProxy.getRunningBehaviors()
        for b in bhv:
            print "Killing already running behavior: " + str(b)
            self.behaviorProxy.stopBehavior(b)

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
