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
MIN = 60

STRAIGHT_W=3
MAX_ANGLE = 95


class NXTLocomotionCommandHandler:
    def __init__(self, proj, shared_data, leftDriveMotor='PORT_B', rightDriveMotor='PORT_C', steeringMotor='none', leftForward=True, rightForward=True):
        """
        Locomotion Command handler for Mindstorms.
        
        leftDriveMotor (str): The motor that drives the left side (default='PORT_B')
        rightDriveMotor (str): The motor that drives the right side (default='PORT_C')
        steeringMotor (str): The motor that controls steering, if applicable (default='none')
        leftForward (bool): Whether the standard forward motion provides forward motion (default=True)
        rightForward (bool): Whether the standard forward motion provides forward motion (default=True)
        """
            
        self.nxt = shared_data['NXT_INIT_HANDLER']
        self.pose = proj.h_instance['pose']
        
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
        
        if(steeringMotor=='none'): 
            self.differentialDrive=True
        if(not self.differentialDrive):
            if(steeringMotor=='PORT_A'): self.steerMotor = Motor(self.nxt.brick, PORT_A)
            if(steeringMotor=='PORT_B'): self.steerMotor = Motor(self.nxt.brick, PORT_B)
            if(steeringMotor=='PORT_C'): self.steerMotor = Motor(self.nxt.brick, PORT_C)
            self.tacho = self.getUsefulTacho(self.steerMotor)
        
    def sendCommand(self, cmd):
        """     
        Send movement command to the NXT similar to the process the nao uses
        """
      
        leftPow = BACK
        if self.leftForward: leftPow = FORTH
        rightPow = BACK
        if self.rightForward: rightPow = FORTH
      
        def forward(sec, power):
            for motor in self.driveMotors:
                motor.run(power)
            sleep(sec)
            for motor in self.driveMotors:
                motor.idle()
                
        def go(power):
            for motor in self.driveMotors:
                motor.run(power)
        
        def carTurn(sec, direction):
            self.steerMotor.run(direction)
            forward(sec,leftPow)            
            self.steerMotor.idle()
        
        def left(sec, power):
            self.left.run(power)
            self.right.run(leftPow)
            sleep(sec)
            self.left.idle()
            self.right.idle()
            
        def right(sec, power):
            self.left.run(rightPow)
            self.right.run(power)
            sleep(sec)
            self.left.idle()
            self.right.idle()
            
        def idle():
            for motor in self.driveMotors:
                motor.idle()
            if(not self.differentialDrive): 
                self.steerMotor.idle()
                    
        def convertToUseful(v, angle):
            self.angle = angle*180/pi
            self.v = v
            while(self.angle>180): self.angle-=360
            while(self.angle<-180): self.angle+=360  
        
        def goToDegree(degree):
            """This function takes the desired angle the motor should be at and gets it there"""
            curDegree = self.getUsefulTacho(self.steerMotor)
            degreeRange = abs(curDegree-degree)
            leftPower = -75.0
            rightPower = 75.0
            powerRange=rightPower-MIN
            while(curDegree>degree+RANGE or curDegree<degree-RANGE):
                reset=True
                while(curDegree>degree+RANGE):
                    if(abs(curDegree-degree)<30): leftPower=-MIN
                    elif(abs(curDegree-degree)>degreeRange): leftPower = -75
                    else: leftPower = -(((abs(curDegree-degree)/degreeRange)*powerRange)+MIN)
                    if reset: print 'leftPower: ' + str(leftPower) + ' curDegree: ' + str(curDegree)
                    reset = False
                    self.steerMotor.run(leftPower)
                    lastDegree=curDegree
                    curDegree=self.getUsefulTacho(self.steerMotor)
                    if(lastDegree==curDegree): break
                    if(self.v==0): break # check for pause...
                self.steerMotor.idle()
                curDegree=self.getUsefulTacho(self.steerMotor)
                reset = True
                while(curDegree<degree-RANGE):
                    if(abs(curDegree-degree)<30): rightPower=MIN
                    elif(abs(curDegree-degree)>degreeRange): rightPower = 75
                    else: rightPower = (((abs(degree-curDegree)/degreeRange)*powerRange)+MIN)
                    if reset: print 'rightPower: ' + str(rightPower) + ' curDegree: ' + str(curDegree)
                    reset = False
                    self.steerMotor.run(rightPower)
                    lastDegree=curDegree
                    curDegree=self.getUsefulTacho(self.steerMotor)
                    if(lastDegree==curDegree): break
                    if(self.v==0): break # check for pause...
                self.steerMotor.idle()
                curDegree=self.getUsefulTacho(self.steerMotor)
                if(self.v==0): break # check for pause...
            self.steerMotor.idle()
            print 'at degree: ' + str(self.getUsefulTacho(self.steerMotor))
    
        def difDrive():
            if(self.v==0):
                idle()
            elif self.angle>5:
                left(1,leftPow*self.angle/360)
            elif self.angle<5:
                right(1,rightPow*self.angle/360)
            else:
                forward(1,leftPow)
        
        def nonDifDrive():
            """
            Methodology behind a car type drive.  It checks current pose and uses given vx and vy to calculate whether it should be turning or not.
            """
            pose = self.pose.getPose() 
            theta = pose[2]*180/pi+90 #Facing Direction
            #Orient facing direction between -180 and 180
            while(theta>180): theta-=360 
            while(theta<-180): theta+=360
            phi = atan2(vy,vx)*180/pi #Direction to POI
            if(self.v==0): #pause command sent
                idle()
            elif(phi+360-theta<theta-phi): #quadrant 3 theta and quadrant 2 phi
                #left
                degree=-MAX_ANGLE
                #print 'theta='+str(theta)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            elif(theta+360-phi<phi-theta): #quadrant 2 theta and quadrant 3 phi
                #right
                degree=MAX_ANGLE
                #print 'theta='+str(theta)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            elif(phi+RANGE<theta): #right turn to line up
                #right
                degree=MAX_ANGLE
                #print 'theta='+str(theta)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            elif(phi-RANGE>theta): #left turn to line up
                #left
                degree=-MAX_ANGLE
                #print 'theta='+str(theta)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            else:                   # general straight direction
                #straight
                degree=self.tacho
                #print 'theta='+str(theta)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            if(self.v!=0):
                go(leftPow*.65)
            
        
        #convertToUseful(cmd[0],cmd[1])
        pose = self.pose.getPose()
        vx=cmd[0]
        vy=cmd[1]
        theta = pose[2]+(pi/2)
        self.v = cos(theta)*vx+sin(theta)*vy # magnitude of v
        self.angle = (1/.6)*(-sin(theta)*vx + cos(theta)*vy)
        if(self.differentialDrive):
            difDrive()
        else:
            #print 'gloabal center: ' + str(self.tacho)
            nonDifDrive()
            
    def getUsefulTacho(self, motor):
        """Turns instance data from tachometer in to useful integer"""
        tacho = tuple(int(n) for n in str(motor.get_tacho()).strip('()').split(','))
        return tacho[0]
            