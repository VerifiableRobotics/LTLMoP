#!/usr/bin/env python
"""
=====================================================
deadReckoningPose.py - Pose Handler for dead reckoning
=====================================================
"""

import sys, time
from numpy import *
from lib.regions import *
from lib.handlers.share.Pose import _pyvicon
from math import pi, sin, cos
import thread
from time import sleep
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C

import lib.handlers.handlerTemplates as handlerTemplates

class DeadReckoningPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data, compareAgainstVicon=False, track=.119, wheelBase=.178, driveWheelTeeth=1, driveMotorTeeth=1, wheelDiameter=.0415, startX=0.0, startY=0.0, startTheta=1.5708):
        """
        Pose handler for dead reckoning

        compareAgainstVicon (bool): Whether Vicon data is to be compared during this simulation (default=False)
        track (float): The distance in meters between the left and right wheels on the robot (default=.119)
        wheelBase (float): The distance in meters between the front and back wheels on the robot (default=.178)
        driveWheelTeeth (int): The teeth on the gear that is directly connected to the drive wheels [1 implies none] (default=1)
        driveMotorTeeth (int): The teeth on the gear attached to the drive motor [1 implies none] (default=1)
        wheelDiameter (float): The diameter of the drive wheels on the robot (default=.0415)
        startX (float): The starting x position of the robot (default=0.0)
        startY (float): The starting y position of the robot (default=0.0)
        startTheta (float): The starting facing angle of the robot (default=1.5708)
        """
        
        # definition of globals
        self.executor = executor
        
        self.x = startX
        self.y = startY
        self.theta = startTheta #start facing in the positive y direction
        self.track=track
        self.wheelBase=wheelBase
        self.wheelDiameter=wheelDiameter
        self.gearRatio=float(driveMotorTeeth/driveWheelTeeth)
        self.base=array([self.x,self.y,self.theta]) #a starting pose for the robot
        self.steerAngle=0 #start with wheels straight on car type bot
            
        print 'Current Pose: '+str(self.x)+','+str(self.y)+','+str(self.theta)
        
        #initialize vicon if you want to compare your data
        self.compareAgainstVicon=compareAgainstVicon
        if (compareAgainstVicon):
            self.s = _pyvicon.ViconStreamer()
            self.s.connect("10.0.0.102",800)
            self.s.selectStreams(['Time', "NXT:NXT <t-X>","NXT:NXT <t-Y>","NXT:NXT <a-Z>"])
            self.s.startStreams()
            while self.s.getData() is None: pass
            self.vPose = self.s.getData()
        
        #to clarify when actuating movement is happening
        self.actuating=False
        
        #clear log files for location writing
        f = open('log.txt','w')
        f.close()
        g = open('raw.txt','w')
        g.close()

    def getPose(self, cached=False):
        
        x=self.x
        y=self.y
        o=orient(self.theta+pi/2) #fix irregular vicon pose

        return array([x, y, o])
        
    def setPose(self, cached=False):
        """
        This is called once at the beginning of locomotion.  It allows for constant feedback 
        from the tachometers on the motors
        """
        #get motor setup from locomotion
        self.loco = self.executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
        self.driveMotors=self.loco.driveMotors
        self.steerMotor=self.loco.steerMotor
        self.direction=self.loco.leftForward
        self.steeringRatio=self.loco.steeringRatio
        self.dd=self.loco.differentialDrive
        self.actuating=True
        #new thread for measuring location
        thread.start_new_thread(positionThread,(self, self.gearRatio, self.wheelDiameter, self.direction))
    
    def killPose(self, cached=False):
        """
        This is in case the position handling thread created in setPose above needs to be killed
        """
        self.actuating=False
        
    def updateSteerAngle(self, curDegree, baseDegree):
        """
        For a car type robot, this function updates the angle of the steering wheels.
        This is mostly only called from locomotion.  
        """
        self.steerAngle+=(curDegree-baseDegree)/self.steeringRatio
        print 'Steering angle is '+str(self.steerAngle)  

def getUsefulTacho(motor):
    """
    The tachometer data from the nxt is not useful in current form, this provides 
    usability for those measurements
    """
    tacho = tuple(int(n) for n in str(motor.get_tacho()).strip('()').split(','))
    return tacho[0] 
    
def positionThread(self, gearRatio, wheelDiameter, direction):
    """
    A separate thread function for updating the position of the robot so that
    the main thread does not have to worry about writing to files and other 
    time consuming tasks
    """
    while self.actuating: #as long as something has not requested to kill this thread function
        if self.dd: # differential drive
            baseLeft=getUsefulTacho(self.driveMotors[0])
            baseRight=getUsefulTacho(self.driveMotors[1])
            sleep(.5)
            curLeft=getUsefulTacho(self.driveMotors[0])
            curRight=getUsefulTacho(self.driveMotors[1])
            if curLeft+curRight-baseRight-baseLeft!=0:
                updatePosDD(self,curLeft,baseLeft,curRight,baseRight,gearRatio,wheelDiameter,direction)
        else: # not differential drive
            baseDegree=getUsefulTacho(self.driveMotors[0])
            sleep(.5)
            curDegree=getUsefulTacho(self.driveMotors[0])
            if curDegree-baseDegree!=0:
                updatePosition(self,curDegree,baseDegree,gearRatio,wheelDiameter,direction)         
    
def updatePosition(self, curDegree, baseDegree, gearRatio, wheelDiameter, direction):
    """
    This function updates the position of the robot using dead reckoning algorithms.  
    The primary tool for this analysis is the tachometer on the motors and the type
    of robot that has been specified. This is for a car-type robot.  
    """
    #distance is the traveled path length
    #curDegree is the tachometer reading at time t(n)
    #baseDegree is the tachometer reading at time t(n-1)
    #distance-the total degrees traveled converted to rotations in wheel times the circumference
    distance=(curDegree-baseDegree)*gearRatio*pi/180*wheelDiameter/2*1.6
    if self.steerAngle==0: #going straight
        if direction:
            self.x+=distance*cos(self.theta)
            self.y+=distance*sin(self.theta)
        else: 
            self.x+=-distance*cos(self.theta)
            self.y+=-distance*sin(self.theta)
    else: #turning
        #r is radius, the turning radius of the arc traveled
        #self.steerAngle is the angle of the front wheels from center (~~~ -60->60 degrees)
        #track is the width of the distance between wheels
        #wheelBase is the length of the distance between wheels
        r=abs(self.track/2*cos(self.steerAngle*pi/180))+abs(self.wheelBase/sin(self.steerAngle*pi/180))
        if direction: #user defined forward direction
            if self.steerAngle<0: #distance/r would be equal to delta theta
                self.theta+=-distance/r
            else:
                self.theta-=-distance/r
        else:
            if self.steerAngle<0:
                self.theta+=distance/r
            else:
                self.theta-=distance/r
        self.theta=orient(self.theta) #fix to -pi to pi
        #adding in deltax and deltay
        #distance/r is dtheta
        #r-rcos(dtheta) or rsin(dtheta) are the transformations assuming theta = 0
        #cos(self.theta) and sin(self.theta) transforms to overall coordinate system
        self.x+=-(r-(r*cos((-distance/r))))*cos(self.theta)*20
        self.y+=r*sin((-distance/r))*sin(self.theta)
    positionLog(self, distance)

def updatePosDD(self,curLeft,baseLeft,curRight,baseRight,gearRatio,wheelDiameter,direction):
    """
    Update position for differential drive type robot using dead reckoning.
    """
    slipConstant=2.5 # generall error summed up as slip
    smallThetaConstant=15 # small numbers get rounded and this fixes that problem
    #curRight/curLeft are the current tachometer readings on the right/left motors in degrees
    #baseRight/baseLeft are the previous tachometer readings on the right/left motors in degrees
    #distance is the path distance of travel
    #dtheta is the change in facing direction from the previous point to the new point
    #distance - take the average of the wheel degrees, convert to rotations and convert to distance
    #dtheta - take the difference between the wheel degrees, convert to distance, divide by the track
    distance=((curRight-baseRight)+(curLeft-baseLeft))/2*gearRatio*pi/180*wheelDiameter/2
    dtheta=((curRight-baseRight)-(curLeft-baseLeft))*pi/360/self.track*wheelDiameter*slipConstant
    if direction:  # user inputed directino of forward movement
        self.theta+=dtheta #change in theta just gets added
        self.theta=orient(self.theta) # keeps theta between -pi and pi
        if dtheta!=0: #turning
            r=distance/dtheta #radius using arc length formula
            if dtheta>0:
                #add in a delta x, cos(self.theta) is the correction for actual coordinate system, r-rcos(dtheta) is the change in position relative to the current theta
                self.x+=(r-r*cos(dtheta))*cos(self.theta)*smallThetaConstant
            else: #for relative coordinate system, correction for signs
                self.x-=(r-r*cos(dtheta))*cos(self.theta)*smallThetaConstant
            #similar to delta x, add in a delta y
            self.y+=r*sin(dtheta)*sin(self.theta)*slipConstant
        else: #going straight
            self.x+=distance*cos(self.theta)
            self.y+=distance*sin(self.theta)
    else:
        self.theta-=dtheta
        if dtheta!=0:
            r=distance/dtheta
            if dtheta>0:
                self.x-=(r-r*cos(dtheta))*cos(self.theta)*smallThetaConstant
            else:
                self.x+=(r-r*cos(dtheta))*cos(self.theta)*smallThetaConstant
            self.y-=r*sin(dtheta)*sin*(self.theta)*slipConstant
        else:
            self.x-=distance*cos(self.theta)
            self.y-=distance*sin(self.theta)
    positionLog(self, distance, dtheta)
    
def positionLog(self, distance, dtheta=0):
    """
    Logs position for comparing dead reckoning against Vicon
    """
    f = open('log.txt','a')
    curLoc = [self.x-self.base[0],self.y-self.base[1],self.theta]
    curLoc[2] = orient(curLoc[2])
    curString = str(curLoc[0])+','+str(curLoc[1])+','+str(curLoc[2])+','
    f.write(curString) #log x,y,theta for dead reckoning
    if self.compareAgainstVicon:
        rawPose = self.s.getData()
        rawPose[3]+=pi/2
        rawPose[3]=orient(rawPose[3])
        self.vPose[3]=orient(self.vPose[3])
        viconPose = (rawPose[1]/1000,rawPose[2]/1000,rawPose[3])
        curVic = [viconPose[0]-(self.vPose[1]/1000),viconPose[1]-(self.vPose[2]/1000),viconPose[2]]
        vString=str(curVic[0])+','+str(curVic[1])+','+str(curVic[2])
        f.write(vString+'\n') #log x,y,theta for vicon
        g = open('raw.txt','a')
        g.write(str(distance)+','+str(self.steerAngle*pi/180))
        g.write(','+vString+','+str(dtheta)+'\n') #log input data to test fromulas
        g.close()
    f.close()

def orient(theta):
    """
    Takes any angle in radians and converts it to an angle between -pi and pi (in radians)
    """
    while theta> pi:theta-=2*pi
    while theta<-pi:theta+=2*pi
    return theta
        
