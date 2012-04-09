#!/usr/bin/env python
"""
===============================================
naoActuator.py - Nao Actuator Handler
===============================================

Control predefined Nao motion besides walking
"""

import time
import threading

class naoActuatorHandler:
    def __init__(self, proj, shared_data):
        """

        """
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']

        self.ledProxy = None
        self.ttsProxy = None
        self.audioProxy = None
        self.audioFileIDs = {}
        self.behaviorProxy = None

        self.doCount = False


    #####################################
    ### Available actuator functions: ###
    #####################################

    def faceLEDs(self, actuatorVal, initial=False):
        """
        Use Nao's face LED to indicate state of proposition.
        """


        if initial:
            if self.ledProxy is None:
                self.ledProxy = self.naoInitHandler.createProxy('ALLeds')
        else:
            if actuatorVal == True:
               self.ledProxy.setIntensity('FaceLeds',1)
            else:
               self.ledProxy.setIntensity('FaceLeds',0)

    def _countingThread(self, start, skip, period):
        number = start
        while 1:
            if self.doCount:
                self.ttsProxy.say(str(number))
                number = number + skip
                time.sleep(period)
            else:
                # Reset the count
                number = start
                time.sleep(0.1)

    def countOutLoud(self, start, skip, period, actuatorVal, initial=False):
        if initial:
            if self.ttsProxy is None:
                self.ttsProxy = self.naoInitHandler.createProxy('ALTextToSpeech')
            countThread = threading.Thread(target = self._countingThread, args = (start, skip, period))
            countThread.start()
        else:
            if actuatorVal == True:
                self.doCount = True
            else:
                self.doCount = False

    def sayPhrase(self, phrase, actuatorVal, initial=False):
        """
        Use Nao's test to speech system to speak a word.

        phrase (string): The word to be spoken
        """
        if initial:
            if self.ttsProxy is None:
                self.ttsProxy = self.naoInitHandler.createProxy('ALTextToSpeech')
        else:
            if actuatorVal == True:
                self.ttsProxy.say(phrase)

    def playSoundFile(self, filename, actuatorVal, initial=False):
        if initial:
            if self.audioProxy is None:
                self.audioProxy = self.naoInitHandler.createProxy('ALAudioPlayer')
            self.audioFileIDs[filename] = self.audProxy.loadFile(filename)
        else:
            if actuatorVal == True:
                self.audProxy.play(self.audioFileIDs[filename])
                # TODO: Make this pause dynamic to the length of the clip
                time.sleep(1)

    def _killBehaviors(self):
        bhv = self.behaviorProxy.getRunningBehaviors()
        for b in bhv:
            print "Killing already running behavior: " + str(b)
            self.behaviorProxy.stopBehavior(b)

    def runBehavior(self, startBehaviorName, endBehaviorName, actuatorVal, initial=False):
        if initial:
            if self.behaviorProxy is None:
                self.behaviorProxy = self.naoInitHandler.createProxy('ALBehaviorManager')
            #self.behaviorProxy.preloadBehavior(startBehaviorName)
            #if endBehaviorName != "":
            #    self.behaviorProxy.preloadBehavior(endBehaviorName)
        else:
            if actuatorVal == True:
                self._killBehaviors()
                self.behaviorProxy.runBehavior(startBehaviorName)
            else:
                if endBehaviorName != "":
                    self._killBehaviors()
                    self.behaviorProxy.runBehavior(endBehaviorName)


