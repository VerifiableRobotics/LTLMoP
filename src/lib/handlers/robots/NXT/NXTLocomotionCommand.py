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
MAX_ANGLE = 105

LOW=0


class NXTLocomotionCommandHandler:
    def __init__(self, proj, shared_data, leftDriveMotor='PORT_B', rightDriveMotor='PORT_C', steeringMotor='none', leftForward=True, rightForward=True):
        """
        Locomotion Command handler for Mindstorms.
        
        leftDriveMotor (str): The motor that drives the left side (default='PORT_B')
        rightDriveMotor (str): The motor that drives the right side (default='PORT_C')
        steeringMotor (str): The motor that controls steering, if applicable (default='none')
        leftForward (bool): Whether forward direction is positive power for the left drive motor (default=True)
        rightForward (bool): Whether forward direction is positive power for the right drive motor (default=True)
        """
            
        self.nxt = shared_data['NXT_INIT_HANDLER'] # shared data is the nxt and its functions in this case
        self.pose = proj.h_instance['pose'] # pose data is useful for travel
        
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
        if(steeringMotor=='none'): 
            self.differentialDrive=True
        if(not self.differentialDrive):
            if(steeringMotor=='PORT_A'): self.steerMotor = Motor(self.nxt.brick, PORT_A)
            if(steeringMotor=='PORT_B'): self.steerMotor = Motor(self.nxt.brick, PORT_B)
            if(steeringMotor=='PORT_C'): self.steerMotor = Motor(self.nxt.brick, PORT_C)
            self.tacho = self.getUsefulTacho(self.steerMotor)
        
        self.once=True        
        
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
                    
        def convertToUseful(v, angle):  #currently unused
            '''Takes turn angle and converts to degrees, globally assigns velocity, and gets angle between -180 and 180'''
            self.angle = angle*180/pi
            self.v = v
            while(self.angle>180): self.angle-=360
            while(self.angle<-180): self.angle+=360  
        
        def goToDegree(degree):
            '''Takes a degree measurement (from the tachometer) and carefully aligns the steering motor to that degree'''
            curDegree = self.getUsefulTacho(self.steerMotor)            #currently angled at this degree
            degreeRange = abs(curDegree-degree)                         #distance in degrees it has to run
            leftPower = -75.0                                           #full power for steering
            rightPower = 75.0
            powerRange=rightPower-MIN                                   #power range for steering, MIN is the minimum power for movement on steering
            while(curDegree>degree+RANGE or curDegree<degree-RANGE):    #checks to see if the current angle is close enough to the desired
                #reset=True
                while(curDegree>degree+RANGE):
                    if(abs(curDegree-degree)<30): leftPower=-MIN        #small angle change necessitates small power
                    elif(abs(curDegree-degree)>degreeRange): leftPower = -75    #large angle change necessitates max power
                    else: leftPower = -(((abs(curDegree-degree)/degreeRange)*powerRange)+MIN)   #As you get closer to the angle, decrease power to steering motor
                    #if reset: print 'leftPower: ' + str(leftPower) + ' curDegree: ' + str(curDegree)
                    #reset = False
                    self.steerMotor.run(leftPower)
                    lastDegree=curDegree
                    curDegree=self.getUsefulTacho(self.steerMotor)      #get new current degree
                    if(lastDegree==curDegree): break                    #implies the motor is stuck
                    if(self.v==0): break                                #check for pause...
                self.steerMotor.idle()                                  #always idle motors before giving power incase opposite direction
                curDegree=self.getUsefulTacho(self.steerMotor)          #recheck current degree
                #reset = True
                while(curDegree<degree-RANGE):                          #Same as above
                    if(abs(curDegree-degree)<30): rightPower=MIN
                    elif(abs(curDegree-degree)>degreeRange): rightPower = 75
                    else: rightPower = (((abs(degree-curDegree)/degreeRange)*powerRange)+MIN)
                    #if reset: print 'rightPower: ' + str(rightPower) + ' curDegree: ' + str(curDegree)
                    #reset = False
                    self.steerMotor.run(rightPower)
                    lastDegree=curDegree
                    curDegree=self.getUsefulTacho(self.steerMotor)
                    if(lastDegree==curDegree): break
                    if(self.v==0): break # check for pause...
                self.steerMotor.idle()
                curDegree=self.getUsefulTacho(self.steerMotor)
                if(self.v==0): break # check for pause...
            self.steerMotor.idle()
            #print 'at degree: ' + str(self.getUsefulTacho(self.steerMotor))
    
        def difDrive():
            '''
            Methodology behind a differential drive.  It uses omega (angle) the turning velocity.
            '''
            angle = self.angle*180/pi
            if angle > 0 or angle < 0:
                print 'angle='+str(angle)+' w='+str(self.angle)
            leftPow = -75                           #max powers
            if self.leftForward: leftPow = 75
            rightPow = -75
            if self.rightForward: rightPow = 75
            if(self.v==0):
                idle()                              #pause...
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
        
        def nonDifDrive():
            '''
            Methodology behind a car type drive.  It checks current pose and uses given vx and vy to calculate whether it should be turning or not.
            '''
            pose = self.pose.getPose() 
            angle = pose[2]*180/pi+90               #Facing Direction
            #Orient facing direction between -180 and 180
            while(angle>180): angle-=360 
            while(angle<-180): angle+=360
            phi = atan2(vy,vx)*180/pi               #Direction to POI

            if(self.v==0):                          #pause command sent
                idle()
            elif(phi+360-angle<angle-phi):          #quadrant 3 angle and quadrant 2 phi
                #left
                degree=-MAX_ANGLE
                #print 'angle='+str(angle)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            elif(angle+360-phi<phi-angle):          #quadrant 2 angle and quadrant 3 phi
                #right
                degree=MAX_ANGLE
                #print 'angle='+str(angle)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            elif(phi+RANGE<angle):                  #right turn to line up
                #right
                degree=MAX_ANGLE
                #print 'angle='+str(angle)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            elif(phi-RANGE>angle):                  #left turn to line up
                #left
                degree=-MAX_ANGLE
                #print 'angle='+str(angle)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            else:                                   #general straight direction
                #straight
                degree=self.tacho
                #print 'angle='+str(angle)+' phi='+str(phi)+' going to: '+str(degree)+' vx='+str(vx)+' vy='+str(vy)
                goToDegree(degree)
            if(self.v!=0):                          #run drive motors
                go(leftPow*.65)
            
        
        #convertToUseful(cmd[0],cmd[1])
        pose = self.pose.getPose()                  #get pose from vicon
        vx=cmd[0]                                   #decompose velocity
        vy=cmd[1]
        theta = pose[2]+(pi)                      #get facing angle (account for vicon axes)
        self.v = cos(theta)*vx+sin(theta)*vy        #magnitude of v
        if self.once:
            self.pose.setPose()
            self.once=False
        if(self.differentialDrive):                 #handles differential drive
            theta=theta-pi                          #orient angle for reverse direction of travel
            #self.angle = (1/.6)*(-sin(theta)*vx + cos(theta)*vy)    # omega
            self.angle = atan2(vy/.29,vx/.29) - theta
            print 'Vx: '+str(vx/.29)+' Vy: '+str(vy/.29)+' theta: '+str(theta)
            while self.angle>pi: self.angle-=2*pi
            while self.angle<-pi: self.angle+=2*pi
            difDrive()
        else:                                       #handles car type drive
            #print 'gloabal center: ' + str(self.tacho)
            nonDifDrive()
            
    def getUsefulTacho(self, motor):
        """Turns instance data from tachometer in to useful integer"""
        # the tachometer data from the nxt is not useful in current form, this provides usability
        tacho = tuple(int(n) for n in str(motor.get_tacho()).strip('()').split(','))
        return tacho[0]
            