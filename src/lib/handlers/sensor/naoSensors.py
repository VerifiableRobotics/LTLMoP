#!/usr/bin/env python
"""
====================================================
naoSensors.py - Sensor handler for the Aldebaran Nao
====================================================
"""

try:
    import naoqi
    from naoqi import ALProxy
except ImportError:
    print "ERROR: Could not find naoqi Python module for interfacing with the Nao.  Is it installed correctly?"
    raise

class naoSensorHandler:
    def __init__(self, proj, shared_data):
        self.memProxy = shared_data['NAO_MEMPROXY']

        self.sttProxy = None
        self.sttVocabulary = []
        self.sttVocabCounter = 0

        self.faceProxy = None

    ###################################
    ### Available sensor functions: ###
    ###################################

    def hearWord(word, threshold, initial=False):
        """
        Use Nao's speech recognition system to detect a spoken word.

        word (string): The word to detect
        threshold (float): Minimum acceptable detection confidence (default=0.2,min=0,max=1)
        """

        if initial:
            ### Initialize speech-to-text

            if self.sttProxy is None:
                # Get proxy
                try:
                    self.sttProxy = ALProxy('ALSpeechRecognition', shared_data['NAO_IP'], shared_data['NAO_PORT'])
                except KeyError, ValueError:
                    print "ERROR: Cannot connect to Nao.  Did you run the Nao initialization handler?"
                    return False

                # Close any previous subscriptions that might have been hanging open
                subs = [x[0] for x in self.sttProxy.getSubscribersInfo()]
                if "ltlmop_sensorhandler" in subs:
                    self.sttProxy.unsubscribe("ltlmop_sensorhandler")

                self.sttProxy.setLanguage("English")
                self.sttProxy.setAudioExpression(False)
                self.sttProxy.setVisualExpression(True)
                self.sttProxy.subscribe("ltlmop_sensorhandler")

            self.sttVocabulary += [word]
            self.sttProxy.setWordListAsVocabulary(self.sttVocabulary)
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


    def seePerson(initial=False):
        """
        Use Nao's face recognition to detect a person's face in the field of view.
        """

        if initial:
            ### Initialize face tracking

            if self.faceProxy is None:
                # Get proxy
                try:
                    self.faceProxy = ALProxy('ALFaceDetection', shared_data['NAO_IP'], shared_data['NAO_PORT'])
                except KeyError, ValueError:
                    print "ERROR: Cannot connect to Nao.  Did you run the Nao initialization handler?"
                    return False

                subs = [x[0] for x in self.faceProxy.getSubscribersInfo()]
                # Close any previous subscriptions that might have been hanging open
                if "ltlmop_sensorhandler" in subs:
                    self.faceProxy.unsubscribe("ltlmop_sensorhandler")
                self.faceProxy.subscribe("ltlmop_sensorhandler")

                return True
        else:
            # Check face detection state
            face_data = self.memProxy.getData("FaceDetected",0)
            return (face_data != [])

    def headTapped(initial=False):
        """
        Check whether the button on top of Nao's head is pressed.
        """

        if initial:
            return True
        else:
            return bool(self.memProxy.getData('FrontTactilTouched',0)) 

