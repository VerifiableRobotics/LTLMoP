#!/usr/bin/env python
"""
==================================================
NXTSensor.py - LEGO Mindstorms NXT Sensor Handler 
==================================================
"""

from nxt.sensor import Light, Sound, Touch, Ultrasonic, Color20
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4

class NXTSensorHandler:
    def __init__(self, proj, shared_data, touchSensor1='PORT_1', touchSensor2='none', colorSensor='PORT_3', ultrasonicSensor='PORT_4'):
        """
        LEGO Mindstorms NXT Sensor handler
        
        touchSensor1 (str): The port connection for the first touch sensor (default='PORT_1')
        touchSensor2 (str): The port connection for the second touch sensor (default='none')
        colorSensor (str): The port connection for the color sensor (default='PORT_3')
        ultrasonicSensor (str): The port connection for the ultrasonic Sensor (default='PORT_4')
        """
        
        self.nxt = shared_data['NXT_INIT_HANDLER']
        
        if(touchSensor1 == 'PORT_1'): self.touch1 = Touch(self.nxt.brick, PORT_1)
        if(touchSensor1 == 'PORT_2'): self.touch1 = Touch(self.nxt.brick, PORT_2)
        if(touchSensor1 == 'PORT_3'): self.touch1 = Touch(self.nxt.brick, PORT_3)
        if(touchSensor1 == 'PORT_4'): self.touch1 = Touch(self.nxt.brick, PORT_4)
        if(touchSensor2 == 'PORT_1'): self.touch2 = Touch(self.nxt.brick, PORT_1)
        if(touchSensor2 == 'PORT_2'): self.touch2 = Touch(self.nxt.brick, PORT_2)
        if(touchSensor2 == 'PORT_3'): self.touch2 = Touch(self.nxt.brick, PORT_3)
        if(touchSensor2 == 'PORT_4'): self.touch2 = Touch(self.nxt.brick, PORT_4)
        if(colorSensor=='PORT_1'): self.color = Color20(self.nxt.brick, PORT_1)
        if(colorSensor=='PORT_2'): self.color = Color20(self.nxt.brick, PORT_2)
        if(colorSensor=='PORT_3'): self.color = Color20(self.nxt.brick, PORT_3)
        if(colorSensor=='PORT_4'): self.color = Color20(self.nxt.brick, PORT_4)
        if(ultrasonicSensor=='PORT_1'): self.ultrasonic = Ultrasonic(self.nxt.brick, PORT_1)
        if(ultrasonicSensor=='PORT_2'): self.ultrasonic = Ultrasonic(self.nxt.brick, PORT_2)
        if(ultrasonicSensor=='PORT_3'): self.ultrasonic = Ultrasonic(self.nxt.brick, PORT_3)
        if(ultrasonicSensor=='PORT_4'): self.ultrasonic = Ultrasonic(self.nxt.brick, PORT_4)  

    ###################################
    ### Available sensor functions: ###
    ###################################
    def feel(self, sensorNumber=1, initial=False):
        """
        Use the touch sensors (pressed = True)
        
        sensorNumber (int): The number of the sensor [1 or 2] (default=1)
        """
        if sensorNumber==1: touch = self.touch1
        if sensorNumber==2: touch = self.touch2
        if initial:
            return False
        else:
            data = touch.get_sample()
            return data
    def detectColor(self, colorValue=3, operator='>', initial=False):
        """
        Use the color sensor to see different colors (1-6) with 6 being dark and 1 being light
        
        colorValue (int): The desired value read by the color sensor [1-6, 6->dark] (default=3)
        operator (str): The operator to perform on the color value ['<','>','='] (default='>')
        """
        if initial:
            return False
        else:
            data = self.color.get_sample()
            if operator=='<':
                return (data<colorValue)
            elif operator=='=':
                return (data==colorValue)
            else:
                return (data>colorValue)
    def see(self, ultrasonicDistance=25, operator='<', initial=False):
        """
        Use the ultrasonic sensor to see obstacles (0-255) with 25 being about a foot away
        
        ultrasonicDistance (int): The distance that the ultrasonic returns true [0-255, 25~1foot] (default=25)
        operator (str): The operator for comparing distance ['<','>','='] (default='<')
        """
        if initial:
            return False
        else:
            data = self.ultrasonic.get_sample()
            if operator=='>':
                return (data>ultrasonicDistance)
            elif operator=='=':
                return (data==ultrasonicDistance)
            else:
                return (data<ultrasonicDistance)