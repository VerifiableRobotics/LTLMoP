#!/usr/bin/env python
"""
====================================================
naoSensors.py - Sensor handler for the Aldebaran Nao
====================================================
"""

import threading
import time
import logging

class naoSensorHandler:
    def __init__(self, proj, shared_data):
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']

        self.sttProxy = None
        self.sttVocabulary = []
        self.sttVocabCounter = 0

        self.faceProxy = None
        self.memProxy = None
        self.sttProxy = None
        self.ldmProxy = None

        self.face_detection_thread = None
        self.face_detection_running = False

    def _stop(self):
        if self.face_detection_thread:
            self.face_detection_running = False
            self.face_detection_thread.join()
    ###################################
    ### Available sensor functions: ###
    ###################################
    def seeLandMark(self,landMark_id,initial=False):
        """
        Use Nao's landmark recognition system to detect radial bar code landmark.
        For info about avaible bar code, refer to http://www.aldebaran-robotics.com/documentation/naoqi/vision/allandmarkdetection.html#allandmarkdetection

        landMark_id (int): The id number of bar code to detect
        """
        if initial:

            # initialize landmark detection
            if self.ldmProxy == None:
                self.ldmProxy = self.naoInitHandler.createProxy('ALLandMarkDetection')
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

            ### Initialize land Mark tracking
            subs = [x[0] for x in self.ldmProxy.getSubscribersInfo()]
            # Close any previous subscriptions that might have been hanging open
            if "ltlmop_sensorhandler" in subs:
                self.ldmProxy.unsubscribe("ltlmop_sensorhandler")
            self.ldmProxy.subscribe("ltlmop_sensorhandler", 100, 0.0)
            return True
        else:
            val = self.memProxy.getData("LandmarkDetected",0)

            if(val and isinstance(val, list) and len(val) == 5):
                # We detected naomarks !
                # For each mark, we can read its shape info and ID.

                # Second Field = array of Mark_Info's.
                markInfoArray = val[1]

                try:
                    # Browse the markInfoArray to get info on each detected mark.
                    for markInfo in markInfoArray:

                        # First Field = Shape info.
                        markShapeInfo = markInfo[0]

                        # Second Field = Extra info (ie, mark ID).
                        markExtraInfo = markInfo[1]

                        #print " width %.3f - height %.3f" % (markShapeInfo[3], markShapeInfo[4])

                        #if float(markShapeInfo[3])>0.05 and float(markShapeInfo[4])>0.05:
                        if landMark_id in markExtraInfo:
                            return True

                except Exception, e:
                    print "Naomarks detected, but it seems getData is invalid. ALValue ="
                    print val
                    print "Error msg %s" % (str(e))
            return False


    def hearWord(self, word, threshold, initial=False):
        """
        Use Nao's speech recognition system to detect a spoken word.

        word (string): The word to detect
        threshold (float): Minimum acceptable detection confidence (default=0.2,min=0,max=1)
        """

        if initial:
            ### Initialize speech-to-text

            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

            if self.sttProxy is None:
                self.sttProxy = self.naoInitHandler.createProxy('ALSpeechRecognition')

            # Close any previous subscriptions that might have been hanging open
            subs = [x[0] for x in self.sttProxy.getSubscribersInfo()]
            if "ltlmop_sensorhandler" in subs:
                self.sttProxy.unsubscribe("ltlmop_sensorhandler")

            self.sttVocabulary += [word]
            self.sttProxy.setWordListAsVocabulary(self.sttVocabulary)

            self.sttProxy.setLanguage("English")
            self.sttProxy.setAudioExpression(False)
            self.sttProxy.setVisualExpression(True)
            self.sttProxy.subscribe("ltlmop_sensorhandler")

            # Reset the speech recognition register manually
            self.memProxy.insertData("WordRecognized", [])

            return True
        else:
            # Check speech recognition state

            wds = self.memProxy.getData("WordRecognized",0)

            # HACK: reset the speech recognition register manually once per vocab-cycle
            self.sttVocabCounter += 1
            if self.sttVocabCounter == len(self.sttVocabulary):
                self.memProxy.insertData("WordRecognized", [])
                self.sttVocabCounter = 0

            for wd, prob in zip(wds[0::2], wds[1::2]):
                if wd == word and prob > threshold:
                    print "Recognized word '%s' with p = %f" % (wd, prob)
                    return True

            return False


    def _faceDetectionWithDelay(self,hold_time):
        while self.face_detection_running:
            face_data = self.memProxy.getData("FaceDetected",0)
            if face_data != []:
                self.seenFaceRecently = True
                self.last_detection_time = time.time()
            if self.seenFaceRecently and (time.time()-self.last_detection_time >= hold_time):
                self.seenFaceRecently = False

    def seePerson(self, hold_time, initial=False):
        """
        Use Nao's face recognition to detect a person's face in the field of view.

        hold_time (float): amount of time that sensor will stay true after last falling edge (default=1.0,min=0.1,max=5.0)
        """

        # TODO: if hold_time is zero, don't bother making a timer
        if initial:
            ### Initialize face tracking
            if self.faceProxy is None:
                self.faceProxy = self.naoInitHandler.createProxy('ALFaceDetection')

            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

            subs = [x[0] for x in self.faceProxy.getSubscribersInfo()]
            # Close any previous subscriptions that might have been hanging open
            if "ltlmop_sensorhandler" in subs:
                self.faceProxy.unsubscribe("ltlmop_sensorhandler")
            self.faceProxy.subscribe("ltlmop_sensorhandler")

            self.seenFaceRecently = False
            self.face_detection_thread = threading.Thread(target=self._faceDetectionWithDelay, args=(hold_time,))
            self.face_detection_running = True
            self.face_detection_thread.start()

            return True

        else:
            logging.debug("Did I see a face? {}".format(self.seenFaceRecently))
            return self.seenFaceRecently            

    def headTapped(self, initial=False):
        """
        Check whether the button on top of Nao's head is pressed.
        """

        if initial:
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')
            return True
        else:
            return bool(self.memProxy.getData('FrontTactilTouched',0))

