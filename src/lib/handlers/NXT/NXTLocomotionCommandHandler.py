#!/usr/bin/env python
"""
========================================================================
NXTLocomotionCommand.py - LEGO Mindstorms NXT Locomotion Command Handler
========================================================================

Send forward, side, and angular velocity commands to the NXT.
"""
from math import pi, atan2, cos, sin

from time import sleep

from nxt.brick import Brick
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C

FORTH = 100
BACK = -100

RANGE = 5
MIN = 65

STRAIGHT_W=3
MAX_ANGLE = 35

LOW=0


import lib.handlers.handlerTemplates as handlerTemplates

class NXTLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    def __init__(self, executor, shared_data, leftDriveMotor='PORT_B', rightDriveMotor='PORT_C', steeringMotor='none', steeringGearRatio=1.0, leftForward=True, rightForward=True):
        """
        Locomotion Command handler for NXT Mindstorms.
        
        leftDriveMotor (str): The motor that drives the left side (default='PORT_B')
        rightDriveMotor (str): The motor that drives the right side (default='PORT_C')
        steeringMotor (str): The motor that controls steering, if applicable (default='none')
        steeringGearRatio (float): The gear ratio on the steering control (default=1.0)
        leftForward (bool): Whether forward direction is positive power for the left drive motor (default=True)
        rightForward (bool): Whether forward direction is positive power for the right drive motor (default=True)
        """
            
        self.nxt = shared_data['NXT_INIT_HANDLER'] # shared data is the nxt and its functions in this case
        self.pose = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler) # pose data is useful for travel
        
        # The following creates a tuple of the drive motors based on user input
        # It also derives the left and right motors as well as a steering motor if used
        # There are currently two modes of drive, differential and non-differential (car)
        self.driveMotors=()
        self.differentialDrive = False
        self.leftForward = leftForward
        self.rightForward = rightForward
        if(leftDriveMotor=='PORT_A' or rightDriveMotor=='PORT_A'): 
            self.driveMotors+=Motor(self.nxt.brick, PORT_A),
            if(leftDriveMotor=='PORT_A'):
                self.left=Motor(self.nxt.brick, PORT_A)
            else:
                self.right=Motor(self.nxt.brick, PORT_A)
        if(leftDriveMotor=='PORT_B' or rightDriveMotor=='PORT_B'): 
            self.driveMotors+=Motor(self.nxt.brick, PORT_B),
            if(leftDriveMotor=='PORT_B'):
                self.left=Motor(self.nxt.brick, PORT_B)
            else:
                self.right=Motor(self.nxt.brick, PORT_B)
        if(leftDriveMotor=='PORT_C' or rightDriveMotor=='PORT_C'): 
            self.driveMotors+=Motor(self.nxt.brick, PORT_C),
            if(leftDriveMotor=='PORT_C'):
                self.left=Motor(self.nxt.brick, PORT_C)
            else:
                self.right=Motor(self.nxt.brick, PORT_C)
        
        self.steerMotor=None
        self.steeringRatio=1
        if(steeringMotor=='none'): 
            self.differentialDrive=True
        if(not self.differentialDrive):
            if(steeringMotor=='PORT_A'): self.steerMotor = Motor(self.nxt.brick, PORT_A)
            if(steeringMotor=='PORT_B'): self.steerMotor = Motor(self.nxt.brick, PORT_B)
            if(steeringMotor=='PORT_C'): self.steerMotor = Motor(self.nxt.brick, PORT_C)
            self.tacho = self.getUsefulTacho(self.steerMotor)
            self.steeringRatio=steeringGearRatio # Global variable fro steering gear ratio
        
        self.once=True # start dead reckoning path record only once    
        
    def sendCommand(self, cmd):
        """     
        Send movement command to the NXT
        """
        # general power specifications
        leftPow = BACK
        if self.leftForward: leftPow = FORTH
        rightPow = BACK
        if self.rightForward: rightPow = FORTH
      
        def forward(sec, power): #currently unused
            '''allows for forward movement with power and for time'''
            for motor in self.driveMotors:
                motor.run(power)
            sleep(sec)
            for motor in self.driveMotors:
                motor.idle()
                
        def go(power):
            '''turns the drive motors on with given power'''
            for motor in self.driveMotors:
                motor.run(power)
        
        def carTurn(sec, direction): #currently unused 
            '''specifies time based steering'''
            self.steerMotor.run(direction)
            forward(sec,leftPow)            
            self.steerMotor.idle()
        
        def left(leftPow, power): 
            '''used for differential drive on left turns'''
            self.left.run(power)
            self.right.run(leftPow)
            
        def right(rightPow, power):
            '''used for differential drive on right turns'''
            self.left.run(rightPow)
            self.right.run(power)
            
        def idle():
            '''sets all motors to idle state'''
            for motor in self.driveMotors:
                motor.idle()
            if(not self.differentialDrive): 
                self.steerMotor.idle()
        
        def goToDegree(degree):
            """
            Takes a degree measurement (from the tachometer) and carefully aligns the steering motor to that degree
            """
            curDegree = self.getUsefulTacho(self.steerMotor)            #currently angled at this degree
            baseDegree=curDegree
            degreeRange = abs(curDegree-degree)                         #distance in degrees it has to run
            leftPower = -75.0                                           #full power for steering
            rightPower = 75.0
            powerRange=rightPower-MIN                                   #power range for steering, MIN is the minimum power for movement on steering
            while(curDegree>degree+RANGE or curDegree<degree-RANGE):    #checks to see if the current angle is close enough to the desired
                while(curDegree>degree+RANGE):
                    if(abs(curDegree-degree)<30): leftPower=-MIN        #small angle change necessitates small power
                    elif(abs(curDegree-degree)>degreeRange): leftPower = -75    #large angle change necessitates max power
                    else: leftPower = -(((abs(curDegree-degree)/degreeRange)*powerRange)+MIN)   #As you get closer to the angle, decrease power to steering motor
                    self.steerMotor.run(leftPower)
                    lastDegree=curDegree
                    curDegree=self.getUsefulTacho(self.steerMotor)      #get new current degree
                    if(lastDegree==curDegree): break                    #implies the motor is stuck
                    if(self.v==0): break                                #check for pause...
                self.steerMotor.idle()                                  #always idle motors before giving power incase opposite direction
                curDegree=self.getUsefulTacho(self.steerMotor)          #recheck current degre
                while(curDegree<degree-RANGE):                          #Same as above
                    if(abs(curDegree-degree)<30): rightPower=MIN
                    elif(abs(curDegree-degree)>degreeRange): rightPower = 75
                    else: rightPower = (((abs(degree-curDegree)/degreeRange)*powerRange)+MIN)
                    self.steerMotor.run(rightPower)
                    lastDegree=curDegree
                    curDegree=self.getUsefulTacho(self.steerMotor)
                    if(lastDegree==curDegree): break
                    if(self.v==0): break # check for pause...
                self.steerMotor.idle()
                curDegree=self.getUsefulTacho(self.steerMotor)
                if(self.v==0): break # check for pause...
            self.steerMotor.idle()
            try:
                self.pose.updateSteerAngle(curDegree,baseDegree)
            except:
                pass
    
        def difDrive():
            """
            Methodology behind a differential drive.  It uses facing direction and needed direction
            """
            stopped=False
            angle = self.angle*180/pi
           # if angle > 0 or angle < 0:
                #print 'angle='+str(angle)+' w='+str(self.angle)
            leftPow = -80                           #max powers
            if self.leftForward: leftPow = 80
            rightPow = -80
            if self.rightForward: rightPow = 80
            try:
                if(self.v==0 and not self.actuator.actuatorMotorOn):
                    idle()                              #pause...
                    stopped=True
            except:
                if(self.v==0):
                    idle()
                    stopped=True
            if stopped:
                pass
            elif angle>.5:                           #left turning arc
                if(leftPow>0): arcPower=((leftPow-LOW)*(90-abs(angle))/90)+LOW # scaled power based on required omega
                else: arcPower=((leftPow+LOW)*(90-abs(angle))/90)-LOW
                left(leftPow,arcPower)
            elif angle<-.5:                           #right tuning arc
                if(rightPow>0): arcPower=((rightPow-LOW)*(90-abs(angle))/90)+LOW
                else: arcPower=((rightPow+LOW)*(90-abs(angle))/90)-LOW
                right(rightPow,arcPower)                
            else:
                go(leftPow)                         #straight
            stopped=False
        
        def nonDifDrive():
            """
            Methodology behind a car type drive.  It checks current pose and uses given vx and vy to calculate whether it should be turning or not.
            """
            pose = self.pose.getPose() 
            angle = pose[2]*180/pi+90               #Facing Direction
            stopped=False
            #Orient facing direction between -180 and 180
            while(angle>180): angle-=360 
            while(angle<-180): angle+=360
            phi = atan2(vy,vx)*180/pi               #Direction to POI
            try:
                if(self.v==0 and not self.actuator.actuatorMotorOn):  #pause command sent
                    idle()
                    stopped=True
            except:
                if(self.v==0):
                    idle()
                    stopped=True
            if stopped:
                pass
            elif(phi+360-angle<angle-phi):          #quadrant 3 angle and quadrant 2 phi
                #left
                degree=-MAX_ANGLE*self.steeringRatio
                goToDegree(degree)
            elif(angle+360-phi<phi-angle):          #quadrant 2 angle and quadrant 3 phi
                #right
                degree=MAX_ANGLE*self.steeringRatio
                goToDegree(degree)
            elif(phi+RANGE<angle):                  #right turn to line up
                #right
                degree=MAX_ANGLE*self.steeringRatio
                goToDegree(degree)
            elif(phi-RANGE>angle):                  #left turn to line up
                #left
                degree=-MAX_ANGLE*self.steeringRatio
                goToDegree(degree)
            else:                                   #general straight direction
                #straight
                degree=self.tacho
                goToDegree(degree)
            if(self.v!=0):                          #run drive motors
                go(leftPow*.65)
        
        stopped=False
        pose = self.pose.getPose()                  #get pose from vicon
        vx=cmd[0]                                   #decompose velocity
        vy=cmd[1]
        if self.leftForward:
            theta = pose[2]-(pi/2)  #get facing angle (account for vicon axes)
        else:
            theta = pose[2]+(pi/2) 
        self.v = cos(theta)*vx+sin(theta)*vy        #magnitude of v
        if self.once:
            try:
                self.pose.setPose()
            except:
                print 'Not setting pose with dead reckoning'
                pass
            self.once=False
        if(self.differentialDrive):                 #handles differential drive
            #theta=theta-pi                          #orient angle for reverse direction of travel
            self.angle = atan2(vy,vx) - theta
            #print 'Vx: '+str(vx)+' Vy: '+str(vy)+' theta: '+str(theta)
            while self.angle>pi: self.angle-=2*pi
            while self.angle<-pi: self.angle+=2*pi
            difDrive()
        else:                                       #handles car type drive
            nonDifDrive()
            
    def getUsefulTacho(self, motor):
        """Turns instance data from tachometer in to useful integer"""
        # the tachometer data from the nxt is not useful in current form, this provides usability
        tacho = tuple(int(n) for n in str(motor.get_tacho()).strip('()').split(','))
        return tacho[0]
            
