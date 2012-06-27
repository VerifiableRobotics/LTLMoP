#!/usr/bin/env python
"""
=====================================================
deadReckoningPose.py - Pose Handler for dead reckoning
=====================================================
"""

import sys, time
from numpy import *
from regions import *
import _pyvicon
from math import pi
import thread

class poseHandler:
    def __init__(self, proj, shared_data, differentialDrive=True, compareAgainstVicon=False, track=.119, wheelBase=.178):
        """
        Pose handler for dead reckoning
        
        differentialDrive (bool): Whether the robot is differential Drive (default=True)
        compareAgainstVicon (bool): Whether Vicon data is to be compared during this simulation (default=False)
        track (float): The distance in meters between the left and right wheels on the robot (default=.119)
        wheelBase (float): The distance in meters between the front and back wheels on the robot (default=.178)
        """
        
        r = 0 # initial region
        #start in center of inital region
        center = proj.rfi.regions[r].getCenter()
        self.x = center[0] 
        self.y = center[1]
        self.theta = pi/2 #start facing in the positive y direction
        self.dd=differentialDrive 
        self.track=track
        self.wheelBase=wheelBase
        self.base=self.getPose() #a starting pose for the robot
            
        print 'Current Pose: '+str(self.x)+','+str(self.y)+','+str(self.theta)
        
        #initialize vicon if you want to compare your data
        self.compareAgainstVicon=compareAgainstVicon
        if (compareAgainstVicon):
            self.s = _pyvicon.ViconStreamer()
            self.s.connect("10.0.0.102",800)
            self.s.selectStreams(['Time', "NXT:NXT <t-X>","NXT:NXT <t-Y>","NXT:NXT <a-Z>"])
            self.s.startStreams()
            while self.s.getData() is None: pass
            self.vPose = s.getData()
        
        #get motor setup from locomotion
        self.loco=proj.h_instance['locomotion']
        self.driveMotors()=self.loco.driveMotors()
        self.steerMotor=self.loco.steerMotor
        
        #get values from actuator that determine position
        ''' this is probably broken '''
        self.act=proj.h_instance['actuator']
        thread.start_new_thread(positionThread,(self, self.act.gearRatio, self.act.wheelDiameter, self.act.direction))

    def getPose(self, cached=False):
        
        x=self.x
        y=self.y
        o=self.theta

        return array([x, y, o])

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
    while True:
        if self.dd:
            baseLeft=getUsefulTacho(self.driveMotors[0])
            baseRight=getUsefulTacho(self.driveMotors[1])
            sleep(.5)
            curLeft=getUsefulTacho(self.driveMotors[0])
            curRight=getUsefulTacho(self.driveMotors[1])
            if curLeft+curRight-baseRight-baseLeft!=0:
                updatePosDD(self,curLeft,baseLeft,curRight,baseRight,gearRatio,wheelDiameter,direction)
        else:
            baseDegree=getUsefulTacho(self.driveMotors[0])
            sleep(.5)
            curDegree=getUsefulTacho(self.driveMotors[0])
            if curDegree-baseDegree!=0:
                updatePosition(self,curDegree,baseDegree,gearRatio,wheelDiameter,direction)         
    
def updatePosition(self, curDegree, baseDegree, gearRatio, wheelDiameter, direction):
    """
    This function updates the position of the robot using dead reckoning algorithms.  
    The primary tool for this analysis is the tachometer on the motors and the type
    of robot that has been specified.
    """
    #distance is the traveled path length
    #curDegree is the tachometer reading at time t(n)
    #baseDegree is the tachometer reading at time t(n-1)
    #distance-the total degrees traveled converted to rotations in wheel times the circumference
    self.steerAngle = self.act.steerAngle
    distance=(curDegree-baseDegree)*gearRatio*pi/180*wheelDiameter/2
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
    Update position for differential drive type robot using dead reckoning
    """
    slipConstant=.7 # generall error summed up as slip
    smallThetaConstant=10 # small numbers get rounded and this fixes that problem
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
        rawPose = self.getData()
        rawPose[3]+=pi/2
        rawPose[3]=orient(rawPose[3])
        self.vPose[3]=orient(self.vPose[3])
        viconPose = (rawPose[1]/1000,rawPose[2]/1000,rawPose[3])
        curVic = [viconPose[0]-(self.vPose[1]/1000),viconPose[1]-(self.vPose[2]/1000),viconPose[2]]
        vString=str(curVic[0])+','+str(curVic[1])+','+str(curVic[2])
        f.write(vString+'\n') #log x,y,theta for vicon
        g = open('raw.txt','a')
        g.write(str(distance)+','+str(self.act.steerAngle*pi/180))
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
        