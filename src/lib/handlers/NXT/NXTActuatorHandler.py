#!/usr/bin/env python
"""
=====================================================
NXTActuator.py - LEGO Mindstorms NXT Actuator Handler
=====================================================
"""

from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
from time import sleep
from math import sin, cos, pi
import thread, threading

MIN = 60

import lib.handlers.handlerTemplates as handlerTemplates

class NXTActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):
        """
        LEGO Mindstorms NXT Actuator Handler
        """
        self.nxt = shared_data['NXT_INIT_HANDLER']
        self.pose = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler) # pose data is useful for dead reckoning
        self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler) 
        
        self.actuatorMotorOn=False
        
    #####################################
    ### Available actuator functions: ###
    #####################################
    
    def runMotorTime(self, actuatorVal, actuatorMotorPorts='PORT_A', actuatorForward=True, actionTime=1.0, actionPower=100, initial=False):
        """
        Runs a specified motor for power [-128 to 127] and time in seconds
        
        actuatorMotorPorts (str): The motor ports that allow the robot to do an action (default='PORT_A')
        actuatorForward (bool): Whether positive power applies forward motion (default=True)
        actionTime (float): The number of seconds to run the action motor (default=1)
        actionPower (int): The power to the actuator motor [0-100] (default=100)
        """
        if not initial: #no initialization
            self.action=()
            ports = actuatorMotorPorts.split('.')
            for port in ports:
                self.action+=Motor(self.nxt.brick,eval(port)),
            if int(actuatorVal) == 1: #a correction so that it doesn't run on startup (thanks Cameron!)
                if actuatorForward: #if the user wants one direction
                    print 'Running motor '+str(ports)+' for '+str(actionTime)+' seconds at '+str(actionPower)+' power'
                    for motor in self.action:
                        motor.run(actionPower)
                    sleep(actionTime)
                    for motor in self.action:
                        motor.idle()
                else:               #if the users wants the other direction
                    print 'Running motor '+str(ports)+' for '+str(actionTime)+' seconds at '+str(actionPower)+' power'
                    for motor in self.action:
                        motor.run(-actionPower)
                    sleep(actionTime)
                    for motor in self.action:
                        motor.idle()
    
    def runMotorDistance(self, actuatorVal, actuatorMotorPorts='PORT_B.PORT_C', distance=.1, power=100, initial=False):
        """
        Runs a motor for a specific distance of travel
        
        actuatorMotorPorts (str): The ports for the actuation motors (default='PORT_B.PORT_C')
        distance (float): Distance the motor should run for (default=.1)
        power (int): The power given to the motor (default=100)
        """
        if not initial:
            self.motors=()
            ports = actuatorMotorPorts.split('.')
            for port in ports:
                self.motors+=Motor(self.nxt.brick,eval(port)),
            neededDegree=distance/self.pose.wheelDiameter*180/pi/self.pose.gearRatio*2
            baseDegree=getUsefulTacho(self.motors[0])
            if int(actuatorVal)==1:
                self.direction=self.pose.direction
                if self.direction:
                    print 'Running motor '+str(ports)+' over '+str(distance)+' meters at '+str(power)+' power'
                    curDegree=getUsefulTacho(self.motors[0])
                    controlledRun(self,self.motors,power,curDegree,neededDegree)
                else:
                    print 'Running motor '+str(ports)+' over '+str(distance)+' meters at '+str(power)+' power'
                    curDegree=getUsefulTacho(self.motors[0])
                    controlledRun(self,self.motors,-power,curDegree,-neededDegree)
                sleep(1)
                curDegree=getUsefulTacho(self.motors[0])
                
                    
    def runMotorAngle(self, actuatorVal, actuatorMotorPorts='PORT_A', actuatorForward=True, angle=30, motorGearTeeth=1, actuatorGearTeeth=1, power=70, initial=False):
        """
        Runs a motor such that an output reaches the desired angle
        
        actuatorMotorPorts (str): The ports for the actuation motors (default='PORT_A')
        actuatorForward (bool): Whether positive power applies forward movement (default=True)
        angle (int): The desired angle of motion (default=30)
        motorGearTeeth (int): The number of teeth on the motor geer [1 implies no geer] (default=1)
        actuatorGearTeeth (int): The number of teeth on the actuator geer [1 implies no geer] (default=1)
        power (int): The power you wish to apply to the motor (default=70)
        """
        if not initial:
            self.angle=()
            ports = actuatorMotorPorts.split('.')
            for port in ports:
                self.angle+=Motor(self.nxt.brick,eval(port)),
            gearRatio = float(actuatorGearTeeth/motorGearTeeth)
            baseDegree=getUsefulTacho(self.angle[0])
            if int(actuatorVal)==1:
                if (actuatorForward and angle>0)or(not actuatorForward and angle<0):
                    print 'Turning Motor '+str(ports)+' '+str(angle)+' degrees at '+str(power)+' power'
                    curDegree=getUsefulTacho(self.angle[0])
                    controlledRun(self,self.angle,power,curDegree,(angle*gearRatio))
                else:
                    print 'Turning Motor '+str(ports)+' '+str(angle)+' degrees at '+str(power)+' power'
                    curDegree=getUsefulTacho(self.angle[0])
                    controlledRun(self,self.angle,-power,curDegree,(angle*gearRatio))
                sleep(1)
                curDegree=getUsefulTacho(self.angle[0])
                self.pose.updateSteerAngle(curDegree, baseDegree)
                    
    def arcTurn(self, actuatorVal, actuatorMotorPorts='PORT_B.PORT_C', power=100, arcAngle=90, arcRadius=.5, initial=False):
        """
        For a differential drive robot, run the drive motors such that it creates an arc 
        with radius arcRadius and through angle arcAngle.  VERY INNACURATE
        
        actuatorMotorPorts (str): The ports for the actuation motors (default='PORT_A.PORT_B')
        power (int): The desired power to apply to the faster motor (default=100)
        arcAngle (int): The angle that you want the robot to arc through in degrees (default=90)
        arcRadius (float): The desired radius of the turning arc (default=.5)
        """
        if not initial:
            self.motors=()
            ports = actuatorMotorPorts.split('.')
            for port in ports:
                self.motors+=Motor(self.nxt.brick,eval(port)),
            distance = arcRadius*arcAngle*pi/180
            ri = arcRadius-self.pose.track/2
            ro = arcRadius+self.pose.track/2
            baseTravel=(getUsefulTacho(self.motors[0])+getUsefulTacho(self.motors[1]))/2
            curTravel = baseTravel
            degrees = (distance/self.pose.wheelDiameter*360/pi/self.pose.gearRatio)
            if int(actuatorVal)==1:
                self.direction=self.pose.direction
                print 'Arcing through '+str(arcAngle)+' at a radius of '+str(arcRadius)+'m'
                if self.direction:
                    while curTravel<baseTravel+abs(degrees):
                        if arcAngle <0:
                            self.motors[0].run(power)
                            self.motors[1].run(power*ri/ro)
                        else:
                            self.motors[1].run(power)
                            self.motors[0].run(power*ri/ro)
                        curTravel = (getUsefulTacho(self.motors[0])+getUsefulTacho(self.motors[1]))/2
                else:
                    while curTravel>baseTravel-abs(degrees):
                        if arcAngle <0:
                            self.motors[0].run(power)
                            self.motors[1].run(power*ri/ro)
                        else:
                            self.motors[1].run(power)
                            self.motors[0].run(power*ri/ro)
                        curTravel = (getUsefulTacho(self.motors[0])+getUsefulTacho(self.motors[1]))/2
                for motor in self.motors:
                    motor.idle() 
                
    def turnOnMotor(self, actuatorVal, actuatorMotorPorts='PORT_A.PORT_B', power=100, initial=False):
        """
        Turns on a given set of motors with specified power
        
        actuatorMotorPorts (str): The ports for the actuation motors (default='PORT_A.PORT_B')
        power (int): The power sent to the set of motors (default=100)
        """
        if not initial:
            self.actuating=True
            self.on=()
            ports=actuatorMotorPorts.split('.')
            for port in ports:
                self.on+=Motor(self.nxt.brick,eval(port)),
            if int(actuatorVal)==1:
                self.direction=self.loco.leftForward
                self.actuatorMotorOn=True
                if self.direction:
                    print 'Turning '+str(ports)+' on with power '+str(power)
                    for motor in self.on:
                        motor.run(power)
                else:
                    print 'Turning '+str(ports)+' on with power '+str(power)
                    for motor in self.on:
                        motor.run(-power)
                        
    def turnOffMotor(self, actuatorVal, actuatorMotorPorts='PORT_A.PORT_B', initial=False):
        """
        Turns off a given set of motors
        
        actuatorMotorPorts (str): The ports for the actuation motors (default='PORT_A.PORT_B')
        """
        if not initial:
            self.actuating=True
            self.off=()
            ports = actuatorMotorPorts.split('.')
            self.actuatorMotorOn=False
            for port in ports:
                self.off+=Motor(self.nxt.brick,eval(port)),
            if int(actuatorVal)==1:
                for motor in self.off:
                    motor.idle()
                    
    def playTones(self, actuatorVal, frequencies=350, durations=500, initial=False):
        """
        Plays a tone on the nxt with set frequency (Hz) and duration (ms)
        
        frequencies (str): A set of frequencies that the nxt can play [0-20k Hz] (default='350.600')
        durations (str): The time to play the frequency in ms (default='500.250')
        """
        if not initial:
            tones = ()
            freqs = frequencies.split('.')
            for freq in freqs:
                tones+=eval(freq),
            times = ()
            durats = durations.split('.')
            for durat in durats:
                times+=eval(durat),
            if int(actuatorVal) == 1:
                for frequency, duration in zip(tones,times):
                    if duration > 0 and frequency > 0:    #real values for freq and duration
                        self.nxt.brick.play_tone(frequency, duration)
                        print 'playing frequency: '+ str(frequency)
                        sleep(float(duration/1000.)+.1)
                    else:
                        if duration <= 0:
                            print 'duration is not a reasonable number'
                        else:
                            print 'frequency is not a reasonable number'
                        
    def playSoundFile(self, actuatorVal, soundFile='none', initial=False): #I'm not sure if this works
        """
        Plays a sound file on the nxt once
        
        soundFile (str): The file name of the sound you want to play (default='none')
        """
        if not initial and int(actuatorVal) == 1:
            if soundFile!='none':
                self.nxt.brick.play_sound_file(False, soundFile+'.rso')
                sleep(1)
            else:
                print 'no file specified to play'
                
def getUsefulTacho(motor):
    """
    The tachometer data from the nxt is not useful in current form, this provides 
    usability for those measurements
    """
    tacho = tuple(int(n) for n in str(motor.get_tacho()).strip('()').split(','))
    return tacho[0]
    

def controlledRun(self, motors, power, current, degrees):
    """
    Given a motor, its current tachometer reading, and a displacement in degrees for the
    motor to turn, this function will bring the motor to that degree in a controlled
    fashion
    """
    curDegree=getUsefulTacho(motors[0])
    if(power>0):
        while(curDegree-current<degrees):
            control=((power-MIN)*(degrees-curDegree+current)/degrees)+MIN
            #print 'power to motors: '+str(control)+' curDegree:'+str(curDegree)+' needed:'+str(degrees)
            for motor in motors:
                motor.run(control)
            lastDegree=curDegree
            curDegree=getUsefulTacho(motors[0])
    else:
        while(curDegree-current>degrees):   
            control=((power+MIN)*(degrees-curDegree+current)/degrees)-MIN
            #print 'power to motors: '+str(control)+' curDegree:'+str(curDegree)+' needed:'+str(degrees)
            for motor in motors:
                motor.run(control)
            lastDegree=curDegree
            curDegree=getUsefulTacho(motors[0])
    for motor in motors:
        motor.idle()
