#!/usr/bin/env python
"""
=====================================================
NXTActuator.py - LEGO Mindstorms NXT Actuator Handler
=====================================================
"""

from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
from time import sleep

class NXTActuatorHandler:
    def __init__(self, proj, shared_data, actuatorMotor='PORT_A', actuatorForward=True):
        """
        LEGO Mindstorms NXT Actuator Handler
        """
        self.nxt = shared_data['NXT_INIT_HANDLER']

        
        
    #####################################
    ### Available actuator functions: ###
    #####################################
    def doAction(self, actuatorVal, actuatorMotor='PORT_A', actuatorForward=True, actionTime=1, actionPower=100, initial=False):
        """
        Runs a specified motor for power and time
        
        actuatorMotor (str): The motor that allows the robot to do an action (default='PORT_A')
        actuatorForward (bool): Should the motor run forward or backward, true=forward (default=True)
        actionTime (float): The number of seconds to run the action motor (default=1)
        actionPower (int): The power to the actuator motor [0-100] (default=100)
        """
        if not initial:
            if(actuatorMotor=='PORT_A'): self.action = Motor(self.nxt.brick, PORT_A)
            if(actuatorMotor=='PORT_B'): self.action = Motor(self.nxt.brick, PORT_B)
            if(actuatorMotor=='PORT_C'): self.action = Motor(self.nxt.brick, PORT_C)
            if actuatorVal:
                if actuatorForward:
                    self.action.run(actionPower)
                    sleep(actionTime)
                    self.action.idle()
                else:
                    self.action.run(-actionPower)
                    sleep(actionTime)
                    self.action.idle()
    def playTone(self, actuatorVal, frequency=350, duration=500, initial=False):
        """
        Plays a tone on the nxt with set frequency and duration
        
        frequency (int): A frequency that the nxt can play [0-20k] (default=350)
        duration (int): The time to play the frequency in ms (default=500)
        """
        if not initial and actuatorVal:
            if duration > 0 and frequency > 0:
                print 'playing frequency: '+ str(frequency)
                self.nxt.brick.play_tone(frequency, duration)
            else:
                if duration <= 0:
                    print 'duration is not a reasonabl number'
                else:
                    print 'frequency is not a reasonable number'
    def playSoundFile(self, actuatorVal, soundFile='none', initial=False):
        """
        Plays a sound file on the nxt once
        
        soundFile (str): The file name of the sound you want to play (default='none')
        """
        if not initial and actuatorVal:
            if soundFile!='none':
                self.nxt.brick.play_sound_file(False, soundFile+'.rso')
                sleep(1)
            else:
                print 'no file specified to play'
        