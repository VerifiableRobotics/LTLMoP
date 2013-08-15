#!/usr/bin/env python
"""
===============================================
naoActuator.py - Nao Actuator Handler
===============================================

Control predefined Nao motion besides walking
"""

import time
import threading
import almath
import math
import time
from naoqi import ALProxy

class naoActuatorHandler:
    def __init__(self, proj, shared_data):
        """

        """
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']

        self.ledProxy = None
        self.ttsProxy = None
        self.motionProxy = None
        self.memoryProxy = None
        self.audioProxy = None
        self.audioFileIDs = {}
        self.ldmProxy = None
        self.behaviorProxy = None

        self.doCount = False
        self.asyncBehaviorFlags = {}
        self.asyncBehaviorThreads = {}


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

    def receivePackage(self,landmarkSize, camera, actuatorVal, initial=False):
        """
        Use Nao's left hand to track a package(bar code) and grab it if head tapped

        landmarkSize (float): The theoretical diameter of the printed landmark in meter (default=0.04)
        camera(int): The camera used for tracking the bar code, 1 for top camera, 2 for bottom (default=1)
        """

        landmarkTheoreticalSize = landmarkSize #in meters
        if int(camera) == 1:
            currentCamera = "CameraTop"
        else:
            currentCamera = "CameraBottom"
        saw_landmark = False

        DEBUG_FLG = False
        elbowX = 0.1
        elbowY = 0.1
        elbowZ = 0.1

        if initial:
            if self.motionProxy is None:
                self.motionProxy = self.naoInitHandler.createProxy('ALMotion')
            if self.memoryProxy is None:
                self.memoryProxy = self.naoInitHandler.createProxy('ALMemory')
            if self.ldmProxy is None:
                self.ldmProxy = self.naoInitHandler.createProxy('ALLandMarkDetection')
                # Subscribe to LandmarkDetected event from ALLandMarkDetection proxy.
                subs = [x[0] for x in self.ldmProxy.getSubscribersInfo()]
                # Close any previous subscriptions that might have been hanging open
                if "getLandmark" in subs:
                    self.ldmProxy.unsubscribe("getLandmark")
                self.ldmProxy.subscribe("getLandmark")
            if self.behaviorProxy is None:
                self.behaviorProxy = self.naoInitHandler.createProxy('ALBehaviorManager')


        elif actuatorVal== True:
            effector   = "LArm"
            space      = 0
            axisMask   = almath.AXIS_MASK_VEL    # just control position
            isAbsolute = True

            dx         =  +0.15      # translation axis X (meters)
            dy         =  +0.03      # translation axis Y (meters)
            dz         =  +0.10      # translation axis Z (meters)
            dwx        =  0.00      # rotation axis X (radians)
            dwy        =  0.00      # rotation axis Y (radians)
            dwz        =  0.00      # rotation axis Z (radians)
            targetPos  = [dx, dy, dz, dwx, dwy, dwz]
            times      = [2.0] # seconds

            self.motionProxy.stiffnessInterpolation('LArm',1.0,1.0)
            self.motionProxy.stiffnessInterpolation('HeadYaw',1.0,1.0)
            self.motionProxy.stiffnessInterpolation('HeadPitch',1.0,1.0)
            self.motionProxy.openHand('LHand')
            self.motionProxy.positionInterpolation(effector, space, targetPos,axisMask, times, isAbsolute)

            names            = ["LWristYaw"]
            angles           = [-120*almath.TO_RAD]
            fractionMaxSpeed = 0.5
            self.motionProxy.setAngles(names,angles,fractionMaxSpeed)

            for i in range(0, 9000):
                time.sleep(0.05)
                markData = self.memoryProxy.getData("LandmarkDetected")

                # Check whether we got a valid output.
                if(markData and isinstance(markData, list) and len(markData) >= 2):
                    # We detected naomarks !
                    saw_landmark = True
                    # Retrieve landmark center position in radians.
                    wzCamera = markData[1][0][0][1]
                    wyCamera = markData[1][0][0][2]

                    # Retrieve landmark angular size in radians.
                    angularSize = (markData[1][0][0][3]+markData[1][0][0][4])/2

                    # Compute distance to landmark.
                    distanceFromCameraToLandmark = landmarkTheoreticalSize / ( 2 * math.tan( angularSize / 2))


                    # Get current camera position in NAO space.
                    transform = self.motionProxy.getTransform(currentCamera, space, True)
                    transformList = almath.vectorFloat(transform)
                    robotToCamera = almath.Transform(transformList)

                    # Compute the rotation to point towards the landmark.
                    cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

                    # Compute the translation to reach the landmark.
                    cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

                    # Combine all transformations to get the landmark position in NAO space.
                    robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform *cameraToLandmarkTranslationTransform

                    if(DEBUG_FLG):
                        print "x " + str(robotToLandmark.r1_c4) + " (in meters)"
                        print "y " + str(robotToLandmark.r2_c4) + " (in meters)"
                        print "z " + str(robotToLandmark.r3_c4) + " (in meters)"

                        result = self.motionProxy.getPosition(effector, space,True)
                        print result
                        print ''

                    x = robotToLandmark.r1_c4
                    y = robotToLandmark.r2_c4
                    z = robotToLandmark.r3_c4

                    names  = ["HeadYaw", "HeadPitch"]
                    #angles  = [math.atan2(y,x), -math.atan2(z-0.1265,math.hypot(x,y))]
                    angles = [wzCamera/2,wyCamera/2]
                    fractionMaxSpeed  = 0.1
                    self.motionProxy.changeAngles(names, angles, fractionMaxSpeed)

                    if y<0.1:
                        newX = x-elbowX
                        newY = y-elbowY
                        newZ = z-elbowZ

                        dist = math.sqrt(math.pow(newX,2)+math.pow(newY,2)+math.pow(newZ,2))
                        mul = dist/0.13
                        newX = newX/mul
                        newY = newY/mul
                        newY = newY/mul

                        x = newX+elbowX
                        y = newY+elbowY
                        z = newZ+elbowZ
                    else:
                        newX = x-0.0
                        newY = y-elbowY
                        newZ = z-elbowZ

                        dist = math.sqrt(math.pow(newX,2)+math.pow(newY,2)+math.pow(newZ,2))
                        mul = dist/0.23
                        newX = newX/mul
                        newY = newY/mul
                        newY = newY/mul

                        x = newX+0.0
                        y = newY+elbowY
                        z = newZ+elbowZ
                    targetPos  = [x,y,z, dwx, dwy, dwz]
                    times      = [1.0] # seconds
                    #targetPos = [0.15,0.027,0.17,0.0,0.0,0.0]
                    #self.motionProxy.openHand('LHand')
                    self.motionProxy.positionInterpolation(effector, space, targetPos,axisMask, times, isAbsolute)
                    #self.motionProxy.setPosition(effector, space, targetPos,0.2,axisMask)
                if bool(self.memoryProxy.getData('FrontTactilTouched',0)) and saw_landmark:
                    self.behaviorProxy.runBehavior('grab')
                    break

        # Subscribe to LandmarkDetected event from ALLandMarkDetection proxy.
        subs = [x[0] for x in self.ldmProxy.getSubscribersInfo()]
        # Close any previous subscriptions that might have been hanging open
        if "getLandmark" in subs:
            self.ldmProxy.unsubscribe("getLandmark")

        self.motionProxy.stiffnessInterpolation('LArm',0.0,1.0)
        self.motionProxy.stiffnessInterpolation('HeadYaw',0.0,1.0)
        self.motionProxy.stiffnessInterpolation('HeadPitch',0.0,1.0)

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
        repeat (bool): choose whether to continuously repeat the startBehavior as long as prop is true [asynchronously] (default=false)
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


