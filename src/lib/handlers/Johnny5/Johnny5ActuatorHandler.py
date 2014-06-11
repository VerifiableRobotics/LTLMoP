#!/usr/bin/env python
"""
=============================================================
Johnny5ActuatorHandler.py - Johnny 5 Robot Actuator Handler
=============================================================
"""
import os
import time
import logging
import globalConfig

import lib.handlers.handlerTemplates as handlerTemplates

class Johnny5ActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):
        """
        Johnny 5 Robot Actuator Handler
        """

        # find johnny5 serial port
        try:
            self.johnny5Serial = shared_data["Johnny5Serial"]
        except:
            logging.exception("No connection to Johnny 5")
            sys.exit(-1)

        # load config info
        self.config = shared_data['DefaultConfig']


    def _runSequencer(self, FileName):
        """
        .csv file is exported form Sequencer project
        Useful data start from second row
        Column 3:18 shows corresponding servo degree(servo #0-15)
        Column 35:50 shows corresponding servo time(servo #0-15)

        Generate servo commands in format:
        #'Servo Num' + P'Servo Val' + T'Time in ms' + \r

        Between each step, sleep for maximum servo Time in that step sequence
        """
        # save all Sequencer .csv files in foler SequencerFiles under Johnny5 folder
        csvFileName = os.path.join(os.path.dirname(__file__),'SequencerFiles',FileName)

        move = [data.strip('\r\n') for data in open(csvFileName)]
        # Convert .csv file into 2D array "move"
        for i in range(len(move)):
            move[i] = move[i].split(';')

        for i in range(1,len(move)):
            # Servo num in column 3:18, corresponding Time in column 35:50
            for j in range(3,19):
                #Servo Value = Min Servo Value + (Desired Degree-Min Servo Degree)/(Max Servo Degree-Min Servo Degree)*(Max Servo Value-Min Servo Value)
                value = self.config[j-3][1]+(((float(move[i][j]))-self.config[j-3][3])/(self.config[j-3][4]-self.config[j-3][3])*(self.config[j-3][2]-self.config[j-3][1]))
                self.johnny5Serial.write('#%d P%d T%d\r' % ((j-3), int(value), int(move[i][j-3+35])))
            # column 35:50 are corresponding servo time in ms
            # [a:b] goes from a to b-1, b not included
            time.sleep(int(max(move[i][35:51]))/1000)

    #####################################
    ### Available actuator functions: ###
    #####################################

    def standUp(self, actuatorVal, initial=False):
        """
        Stand up with all servos set to default position
        """
        if initial:
            pass
        else:
            if actuatorVal == True:
                self._runSequencer('StandUp.csv')

    def takeBow(self, actuatorVal, initial=False):
        """
        Take a bow if actuatorVal is true
        """
        if initial:
            pass
        else:
            if actuatorVal == True:
                self._runSequencer('TakeBow.csv')

    def highFive(self, actuatorVal, initial=False):
        """
        High five if actuatorVal is true
        """
        if initial:
            pass
        else:
            if actuatorVal == True:
                self._runSequencer('HighFive.csv')
            else:
                self._runSequencer('StandUp.csv')

    def liftArm(self, actuatorVal, arm, initial=False):
        """
        lift up/put down Johnny 5 arm using designated arm when the actuatorVal is True/False

        arm (string): The arm to use, left or right
        """
        if initial:
            pass
        else:
            # lift designated arm
            if actuatorVal == True:
                # send out servo commands
                if arm=='left':
                    # lift left arm
                    self.johnny5Serial.write('#3 P2200 T1000\r')
                    self.johnny5Serial.write('#5 P2200 T1000\r')
                    self.johnny5Serial.write('#6 P1200 T1000\r')
                elif arm=='right':
                    # lift right arm
                    self.johnny5Serial.write('#8 P1000 T1000\r')
                    self.johnny5Serial.write('#10 P500 T1000\r')
                    self.johnny5Serial.write('#11 P2000 T1000\r')
                else:
                    raise ValueError('Cannot recognize arm with value {!r}'.format(arm))
            # put down designated arm
            else:
                # send out servo commands
                if arm=='left':
                    # put down left arm
                    self.johnny5Serial.write('#3 P1576 T1000\r')
                    self.johnny5Serial.write('#5 P1294 T1000\r')
                    self.johnny5Serial.write('#6 P1636 T1000\r')
                elif arm=='right':
                    # put down right arm
                    self.johnny5Serial.write('#8 P1523 T1000\r')
                    self.johnny5Serial.write('#10 P1388 T1000\r')
                    self.johnny5Serial.write('#11 P1465 T1000\r')
                else:
                    raise ValueError('Cannot recognize arm with value {!r}'.format(arm))

            # Pause to let the action complete, will block the locomotion cmd
            # TODO: make this non-blocking
            time.sleep(1)


    def closeHand(self, actuatorVal, hand, initial=False):
        """
        Close Johnny 5 hand using designated hand, open the hand if actuatorVal is False

        hand (string): The hand to use, left or right
        """
        if initial:
            pass
        else:
            # close designated hands
            if actuatorVal==True:
                if hand=='left':
                    # servo value when left hand is fully closed
                    # the cmd includs #(servo_number) P(servo_value) T(time_span)
                    self.johnny5Serial.write('#7 P1700 T3000\r')
                elif hand=='right':
                    # servo value when right hand is fully closed
                    self.johnny5Serial.write('#12 P1300 T3000\r')
                else:
                    raise ValueError('Cannot recognize hand with value {!r}'.format(hand))

            # open up designated hands
            else:
                if hand=='left':
                    # servo value when left hand is fully opened
                    self.johnny5Serial.write('#7 P1100 T3000\r')
                elif hand=='right':
                    # servo value when right hand is fully opened
                    self.johnny5Serial.write('#12 P1800 T3000\r')
                else:
                    raise ValueError('Cannot recognize hand with value {!r}'.format(hand))

            # Pause to let the action complete, will block the locomotion cmd
            # TODO: make this non-blocking
            time.sleep(3)

