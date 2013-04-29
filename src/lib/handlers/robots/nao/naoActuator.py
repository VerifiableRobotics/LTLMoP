#!/usr/bin/env python
"""
===============================================
naoActuator.py - Nao Actuator Handler
===============================================
Control predefined Nao motion besides walking
"""
import time
import threading
import numpy as np
import NaoArmPathPlanner

import sys

class naoActuatorHandler:
    def __init__(self, proj, shared_data):
        """
        """
        
        self.testCode = False
        
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']
        self.sensorData = shared_data["SENSOR_DATA"]
        if not self.testCode == True:
            self.sensor_handler = proj.h_instance['sensor']
        self.ledProxy = None
        self.ttsProxy = None
        self.audioProxy = None
        self.audioFileIDs = {}
        self.behaviorProxy = None
        self.motionProxy= None
        self.trackerProxy = None
        self.doCount = False
        self.asyncBehaviorFlags = {}
        self.asyncBehaviorThreads = {}
        self.armPlanner = NaoArmPathPlanner.PathPlanner(safetyRadius = 0, \
                    stepRange = [10,30], stepInt = 5, attrPow = 2, closeEnough = 10)
     
    """              
    def _stop(self):
        if not self.trackerProxy == None and  self.motionProxy == None:
            self.startActiveTracking(False, False)
        sys.exit(0)
    """   
    
    #####################################
    ### Available actuator functions: ###
    #####################################
    def LEDs(self, FaceLED, EarLED, BrainLED, ChestLED, FeetLED, actuatorVal, initial=False):
        """
        Use Nao's face LED to indicate state of proposition.
        FaceLED (bool): if the eye leds are used. (default=False)
        EarLED (bool): if the ear leds are used. (default=False)
        BrainLED (bool): if the head leds are used. (default=False)
        ChestLED (bool): if the chest led is used. (default=False)
        FeetLED (bool): if the feet leds are used. (default=False)
        """

        if initial:
            if self.ledProxy is None:
                self.ledProxy = self.naoInitHandler.createProxy('ALLeds')
        else:
            if actuatorVal == True:
                if FaceLED == True:
                    self.ledProxy.setIntensity('FaceLeds',1)
                if EarLED == True:
                    self.ledProxy.setIntensity('EarLeds',1)
                if BrainLED == True:
                    self.ledProxy.setIntensity('BrainLeds',1)
                if ChestLED == True:
                    self.ledProxy.setIntensity('ChestLeds',1)
                if FeetLED == True:
                    self.ledProxy.setIntensity('FeetLeds',1)

            else:
                if FaceLED == True:
                    self.ledProxy.setIntensity('FaceLeds',0)
                if EarLED == True:
                    self.ledProxy.setIntensity('EarLeds',0)
                if BrainLED == True:
                    self.ledProxy.setIntensity('BrainLeds',0)
                if ChestLED == True:
                    self.ledProxy.setIntensity('ChestLeds',0)
                if FeetLED == True:
                    self.ledProxy.setIntensity('FeetLeds',0)
                    
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
    def runBehavior(self, startBehaviorName, endBehaviorName, repeat, repeat_period, actuatorVal, initial=False):
        """
        Run a behavior that has been pre-loaded onto Nao with Choregraphe.
        startBehaviorName (string): name of behavior to run when prop goes from False to True
        endBehaviorName (string): name of [optional] behavior to run when prop goes from True to False (default="")
        repeat (bool): choose whether to continuously repeat the startBehavior as long as prop is true [asynchronously] (default=False)
        repeat_period (float): period with which to repeat the action, if `repeat` is True.  Must be longer than the action length.
        """
        if initial:
            if self.behaviorProxy is None:
                self.behaviorProxy = self.naoInitHandler.createProxy('ALBehaviorManager')
            # Preload behaviors to make sure they execute quickly
            self.behaviorProxy.preloadBehavior(startBehaviorName)
            if endBehaviorName != "":
                self.behaviorProxy.preloadBehavior(endBehaviorName)
            if repeat:
                self.asyncBehaviorFlags[startBehaviorName+","+endBehaviorName] = False
                def actionThread(self):
                    while True:
                        if self.asyncBehaviorFlags[startBehaviorName+","+endBehaviorName]:
                            self._killBehaviors()
                            self.behaviorProxy.post.runBehavior(startBehaviorName)
                            time.sleep(repeat_period)
                        else:
                            time.sleep(0.1)
                self.asyncBehaviorThreads[startBehaviorName+","+endBehaviorName] = threading.Thread(target = actionThread, args = (self,))
                self.asyncBehaviorThreads[startBehaviorName+","+endBehaviorName].start()
        else:
            if actuatorVal == True:
                if repeat:
                    self.asyncBehaviorFlags[startBehaviorName+","+endBehaviorName] = True
                else:
                    self._killBehaviors()
                    self.behaviorProxy.runBehavior(startBehaviorName)
            else:
                if repeat:
                    self.asyncBehaviorFlags[startBehaviorName+","+endBehaviorName] = False
                self._killBehaviors()
                if endBehaviorName != "":
                    self.behaviorProxy.runBehavior(endBehaviorName)
    def _moveHandAlongPath(self, path):
        """
        Move the hand along the desired path
        path (nx6 list): A list of waypoints for the arm in degrees
        """
        fracSpeed = .15
        for pos in path:
            pos = [np.radians(x) for x in pos]
            self.motionProxy.angleInterpolationWithSpeed("RArm", pos, fracSpeed)
            
    def startActiveTracking(self, actuatorVal, initial=False):
        """
        Start the NAOs built in active head tracking of the red ball
        """
        consecThresh = 5        # The required number of consecutive detections for True
        zOffset = 40            # The offset from coordinate origins
        if initial:
            if self.trackerProxy == None:
                self.trackerProxy = self.naoInitHandler.createProxy('ALRedBallTracker')
            if self.motionProxy is None:
                self.motionProxy = self.naoInitHandler.createProxy('ALMotion')
                
        else:
            if actuatorVal == True:
                self.trackerProxy.startTracker()
                self.motionProxy.setStiffnesses("Head", 1)
            else:
                self.trackerProxy.stopTracker()
                self.motionProxy.angleInterpolationWithSpeed("Head", [0,0], .3)
                self.motionProxy.setStiffnesses("Head", 0)
    def moveHandTo(self, x, y, z, actuatorVal, initial=False):
        """
        Move the hand to the desired position relative to the center of the chest sphere
        x (float): the x coordinate in mm, forward
        y (float): the y coordinate in mm, left
        z (float): the z coordinate in mm, up
        """
        if initial:
            if self.motionProxy is None:
                self.motionProxy = self.naoInitHandler.createProxy('ALMotion')
        else:
            if actuatorVal == True:
                position = [x, y, z]
                bodyAngles = self.motionProxy.getAngles("Body", True)
                bodyAngles = [np.degrees(x) for x in bodyAngles]
                path = self.armPlanner.pathToPosition(bodyAngles, position)
                if path != None:
                    # Add in the wrist values and the current grasping state of hand
                    graspVal = bodyAngles[-1]
                    path = [(pos + [0, graspVal]) for pos in path]
                    self._moveHandAlongPath(path)
    def grabRedBall(self, radius, actuatorVal, initial=False):
        """
        Grasp the red ball from the location at which it was last detected if possible
        radius (float): The radius of the ball in mm
        """
        fWristAng = 100     # The angle of the wrist upon returning
        if initial:
            if self.motionProxy is None:
                self.motionProxy = self.naoInitHandler.createProxy('ALMotion')
                
            if not self.testCode == True:
                self.startActiveTracking(True, True)
                self.sensor_handler['MAE'].seeRedBall(5, True)

             
        else:
            if actuatorVal == True:
                if self.testCode == True:
                    redBallData = self.sensorData["redBallData"]  
                    
                else:
                    path = None
                    try:
                        while path == None:
                            self.startActiveTracking(True, False)   
                            self.sensor_handler['MAE'].seeRedBall(5, False)
                            #redBallData = self.sensor_handler['MAE']._getRedBallPose()
                            previous_redBallData = self.sensorData["redBallData"]
                            redBallConsistencyCount = 0    
                            redBallConsistencyRadiusThres = 5 # radius allowed for the movement of the ball  
                             
                            while redBallConsistencyCount < 5:#len(redBallData['pos']) == 0:
                                
                                self.sensor_handler['MAE'].seeRedBall(5, False)
                                #redBallData = self.sensor_handler['MAE']._getRedBallPose()
                                redBallData = self.sensorData["redBallData"]
                                print redBallData
                                if len(redBallData['pos']) == 0:
                                    pass
                                else:
                                    if abs(previous_redBallData['pos'][0] - redBallData['pos'][0]) <= redBallConsistencyRadiusThres:
                                        if abs(previous_redBallData['pos'][1] - redBallData['pos'][1]) <= redBallConsistencyRadiusThres:
                                            if abs(previous_redBallData['pos'][2] - redBallData['pos'][2]) <= redBallConsistencyRadiusThres:
                                                redBallConsistencyCount += 1
                                            else:
                                                redBallConsistencyCount = 0
                                        else:
                                            redBallConsistencyCount = 0 
                                    else:
                                        redBallConsistencyCount = 0
                                        
                                    previous_redBallData = redBallData
                                time.sleep(.5)
                        
                            
                            ballObject = NaoArmPathPlanner.Shape3D(redBallData["pos"], radius, \
                                                                    "sphere")
                            bodyAngles = self.motionProxy.getAngles("Body", True)
                            bodyAngles = [np.degrees(x) for x in bodyAngles]
                            path = self.armPlanner.grabBallMotion(bodyAngles, ballObject)
                            
                    except (KeyboardInterrupt, SystemExit):
                        self.startActiveTracking(False, False)
                        print "killing GrabRedBall while loop (again)"
                
                print path
                self.motionProxy.setStiffnesses("Body", 1)
                # Add the hand open values
                pathTo = [(pos + [90]) for pos in path]
                self._moveHandAlongPath(pathTo)
                # Close hand
                self.motionProxy.angleInterpolationWithSpeed("RHand", 0, .1)
                pathFrom = [(pos[0:4] + [fWristAng, 0]) for pos in reversed(path)]
                self._moveHandAlongPath(pathFrom)
                print pathFrom
                
                self.startActiveTracking(False, False)
