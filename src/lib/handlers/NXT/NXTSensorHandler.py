#!/usr/bin/env python
"""
==================================================
NXTSensor.py - LEGO Mindstorms NXT Sensor Handler 
==================================================
"""

from nxt.sensor import Light, Sound, Touch, Ultrasonic, Color20
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
from math import pi

import lib.handlers.handlerTemplates as handlerTemplates

class NXTSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        """
        LEGO Mindstorms NXT Sensor handler
        """
        
        self.nxt = shared_data['NXT_INIT_HANDLER'] 
        self.pose = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler) 
        
            
    ###################################
    ### Available sensor functions: ###
    ###################################
    
    def feel(self, touchPort='PORT_1', initial=False):
        """
        Use the touch sensors (pressed = True)
        
        touchPort (str): The port number of the touch sensor(default='PORT_1')
        """
        
        touch = Touch(self.nxt.brick, eval(touchPort))
        if initial:
            return False                #don't return true until actually checked sensor value
        else:
            data = touch.get_sample()   #already in boolean format
            if data: print 'Touch Sensor '+str(touchPort)+' pressed' 
            return data
            
    def detectColor(self, colorPort='PORT_3', colorValue=3, operator='>', initial=False):
        """
        Use the color sensor to see different colors (1-6) with 6 being black and 1 being white
        
        colorPort (str): The port used for the color sensor (default='PORT_3')
        colorValue (int): The desired value read by the color sensor [1-6, 6->black] (default=3)
        operator (str): The operator to perform on the color value ['<','>','=','!='] (default='>')
        """
        
        color = Color20(self.nxt.brick, eval(colorPort))
        if initial:
            return False                    #don't return true until actually checked sensor value
        else:
            data = color.get_sample()                    #integer between 1 and 6
            output = operation(data,operator,colorValue) #use user input to determine 'true' value
            if output:
                print 'ColorValue is '+operator+' '+str(colorValue)
            return output
            
    def see(self, ultrasonicPort='PORT_4', ultrasonicDistance=25, operator='<', initial=False):
        """
        Use the ultrasonic sensor to see obstacles (0-255) with 25 being about a foot away
        
        ultrasonicPort (str): The port used for the ultrasonic sensor (default='PORT_4')
        ultrasonicDistance (int): The distance that the ultrasonic returns true [0-255, 25~1foot] (default=25)
        operator (str): The operator for comparing distance ['<','>','=','!='] (default='<')
        """
        
        ultrasonic=Ultrasonic(self.nxt.brick, eval(ultrasonicPort))
        if initial:
            return False                    #don't return true until actually checked sensor value
        else:
            data = ultrasonic.get_sample()  #integer between 0 and 255
            output = operation(data,operator,ultrasonicDistance)
            if output:
                print 'Ultrasonic Distsance is '+operator+' '+str(ultrasonicDistance)
            return output

    def tachometer(self, motorPort='PORT_A', degree=0, operator='=', initial=False):
        """
        Use the tachometer in the motors to determine True/False
        
        motorPort (str): The port for the motor that you want to read (default=PORT_A)
        degree (int): The value you are comparing the tachometer value to (default=0)
        operator (str): How the value is being compared to the tachometer value ['<','>','=','!='] (default='=')
        """
        
        if initial:
            return False
        else:
            motor = Motor(self.nxt.brick,eval(motorPort))
            data = getUsefulTacho(motor) #minus large number to positive large number
            output = operation(data,operator,degree) #get bolean based on user input
            if output:
                print 'Tachometer value is '+operator+' '+str(degree)
            return output
    
    def facingDirection(self, angle=90, operator='>', initial=False):
        """
        Take the current theta from pose as sensor input.  Useful primarily for dead reckoning in single region operation.  
        
        angle (int): The angle for comparing the current theta pose to (default=90)
        operator (str): How the angle is being compared to the pose value ['<','>','=','!='] (default='>')
        """
        if initial:
            return False
        else:
            data = self.pose.getPose()[2]*180/pi
            output = operation(data,operator,angle)
            if output:
                print 'Current facing angle is '+operator+' '+str(angle)
            return output
            
            
def getUsefulTacho(motor):
    # the tachometer data from the nxt is not useful in current form, this provides usability
    tacho = tuple(int(n) for n in str(motor.get_tacho()).strip('()').split(','))
    return tacho[0]
    
def operation(data, operator, value): # creates boolean values given operators and data
    if operator=='<':
        return data<value
    elif operator=='!=':
        return data!=value
    elif operator=='==':
        return data==value
    else:  
        return data>value

        
