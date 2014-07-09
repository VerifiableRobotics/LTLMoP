#!/usr/bin/env python
"""
=========================================
iRobotCreateActuator.py - Actuator Handler for iRobotCreate
=========================================
"""

import time
from struct import pack,unpack
from threading import Thread, Event
from numpy import *
from scipy.linalg import norm


import lib.handlers.handlerTemplates as handlerTemplates

class IRobotCreateActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):

        self.iRobotCommunicator = shared_data['robocomm']
        self.pose_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler)
        self.drive_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler)
        self.TakeOver = _TakeOverMotionController(executor, self, shared_data)
        self.hsub = executor.hsub


    #####################################
    ### Available actuator functions: ###
    #####################################
    def sing(self,actuatorVal,initial=False):
        """
        plays the first stored song in the Create
        """
        if initial is False:

            if actuatorVal == True:
                data = pack('BB',141,1)
                self.iRobotCommunicator.broadcastToCreate(data)
                time.sleep(0.5)

    def flash(self,actuatorVal,initial=False):
        """
        flashes the power LED once
        """
        if initial is False:

            if actuatorVal == True:
                data_on = pack('BBBB',139,9,128,255)
                self.iRobotCommunicator.broadcastToCreate(data_on)
                time.sleep(0.25)
            else:
                data_off = pack('BBBB',139,0,0,255)
                self.iRobotCommunicator.broadcastToCreate(data_off)
                time.sleep(0.05)

    def pickup(self,actuatorVal,initial=False):
        if initial:
            # reset the arm so it doesn't jam up
            # make sure the robot has stopped
            pose = self.pose_handler.getPose()
            self.drive_handler.setVelocity(0,0,pose[2])
            print 'POSE:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(pose)))

            data_reset = pack('BBBBBBBBBBBBB',3,0,0,0,48,48,48,48,48,48,48,48,2)
            self.iRobotCommunicator.broadcastToBeagle(data_reset)
            time.sleep(0.05)
        else:
            if actuatorVal == True: #pick it up!
                data_pickup = pack('BBBBBBBBBBBBB',3,0,0,0,48,48,48,48,48,48,48,48,1)
                self.iRobotCommunicator.broadcastToBeagle(data_pickup)
                time.sleep(20)


    def drop(self,actuatorVal,initial=False):
        if initial is False:

            if actuatorVal == True: #drop it!
                # make sure the robot has stopped
                pose = self.pose_handler.getPose()
                self.drive_handler.setVelocity(0,0,pose[2])
                print 'POSE:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(pose)))

                data_reset = pack('BBBBBBBBBBBBB',3,0,0,0,48,48,48,48,48,48,48,48,2)
                self.iRobotCommunicator.broadcastToBeagle(data_reset)
                time.sleep(0.05)

                data_drop = pack('BBBBBBBBBBBBB',3,0,0,0,48,48,48,48,48,48,48,48,0)
                self.iRobotCommunicator.broadcastToBeagle(data_drop)
                time.sleep(12)
            else:
                # print 'RESET IT!'
                # reset the arm so it doesn't jam up
                data_reset = pack('BBBBBBBBBBBBB',3,0,0,0,48,48,48,48,48,48,48,48,2)
                self.iRobotCommunicator.broadcastToBeagle(data_reset)
                time.sleep(0.05)

    def goToMarker(self,actuatorVal,initial=False):
        """
        sends create to the marker location and allows create to dump the object
        at point
        """
        if initial is False:

            if actuatorVal==True: # go to the closest marker and pick it up

                marker = self.iRobotCommunicator.lockOnTargetMarker()
                if marker is not None:
                    print 'ACTUATION IS NOW TAKING OVER MOTION CONTROL'
                    print 'Going for ',array(marker)
                    print 'marker:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(marker)))
                    self.TakeOver.run(marker[0],marker[1])
            else:

                self.TakeOver.stop()
    def checkTakeOver(self):
        return self.TakeOver.isRunning() and not self.TakeOver.close.isSet()



class _TakeOverMotionController(Thread):
    def __init__(self,executor,actuator,shared_data):
        super(_TakeOverMotionController,self).__init__()

        self.iRobotCommunicator = shared_data['robocomm']
        self.pose_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler)
        self.drive_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler)
        self.actuator = actuator
        self.goal = array([])
        self.completed = False
        self.pickUpAttempt = 1
        self.pose = []
        self.close = Event()
        self.hsub= hsub


    def run(self,pos_x,pos_y):
        self.goal = array([pos_x,pos_y])
        #self.dump = array([dump_x,dump_y])
        self.oldPose = self.pose_handler.getPose()
        self.iRobotCommunicator.setHasArrived(False)
        #while self.completed is False:

        print '(TAKEOVER THREAD)going to the pickup location...attemp #',self.pickUpAttempt
        time.sleep(1)
        #print 'Current Pose: ',array(self.oldPose[0:2])
        self.goToPoint(self.goal)
        self.pose = self.pose_handler.getPose()
        print '(TAKEOVER THREAD)beginning pick up...'
        time.sleep(3)
        self.drive_handler.setVelocity(0,0,self.pose[2])
        time.sleep(2)
        #self.pickUpStuff()
        time.sleep(1) # wait for vicon to update
        #failedAttempt,newGoal = self.checkPickUpFail(self.goal)
        #self.completed = not failedAttempt
        self.iRobotCommunicator.setHasArrived(True)
        """
        self.pickUpAttempt +=1
            print '(TAKEOVER THREAD) mission finished? ',self.completed
            time.sleep(0.5)
            if self.completed:
                print '(TAKEOVER THREAD) going to the dump!'
                self.goToPoint(self.dump,True)
                time.sleep(0.5)
                print '(TAKEOVER THREAD) dump it!'
                self.cleanUp()
                time.sleep(0.5)
                print '(TAKEOVER THREAD) Go back to where I started!'
                self.goToPoint(array(self.oldPose[0:2]),True)
                self.iRobotCommunicator.setTakeOver(False)
            else:
                self.goal = newGoal
                print 'Updated marker position: ',self.goal
       """



    def isRunning(self):
        return self.completed is False


        # do everything you want here

    def stop(self):
        #print '(TAKEOVER THREAD) stopping'
        self.completed = False
        self.pickUpAttempt = 1
        self.close.set()

    def goToPoint(self,goal):

        multiplier = 0.3
        pose = self.pose_handler.getPose()

        dist = norm(goal-array(pose[0:2]))
        while dist > 0.23 or dist <= 0.2:
            # update current pose on the map
            print 'POSE:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(pose)))
            print 'marker:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(goal)))
            if dist > 0.19:
                Vel = (goal-array(pose[0:2]))/norm(goal-array(pose[0:2]))
                # send the velocity to the drive handler
                #print 'forward',(goal-array(pose[0:2]))
            else:
                # oops, I got too close to the marker! Let's back out
                Vel = (array(pose[0:2])-goal)/norm(array(pose[0:2])-goal)
                # send the ve locity to the drive handler

                #print 'backout',(goal-array(pose[0:2]))

            self.drive_handler.setVelocity(multiplier*Vel[0],multiplier*Vel[1],pose[2])
            #print 'pose ',[goal[0], goal[1],pose[0],pose[1],pose[2]]
            #print 'VEL: ',(Vel[0],Vel[1])

            pose = self.pose_handler.getPose()
            dist = norm(goal-array(pose[0:2]))
            #print 'dist ', dist
            time.sleep(0.2)

        # adjust angle

        print '(TAKEOVER THREAD) Okey close enough, adjust the angle now'
        self.turnInPlace(goal)

    def checkPickUpFail(self,goal):
        """
        Function first backs the robot out of the pickup location and checks vicon
        to see if there still markers located within a 2cm of the pickup location

        TODO: Vicon seems to jitter sometimes and checking to see if the locations are extact may
        not return the correct value, for now instead of checking the extact point,
        check for markers within a certain radius
        """
        pose = self.pose_handler.getPose()
        # first back out a bit and see if the marker is still there
        Vel = (array(pose[0:2])-goal)/norm(array(pose[0:2])-goal)
        # send the velocity to the drive handler
        self.drive_handler.setVelocity(Vel[0],Vel[1],pose[2])
        time.sleep(2)
        self.drive_handler.setVelocity(0,0,pose[2])
        markers = self.iRobotCommunicator.getViconMarkers()
        stillThere = False
        updatedLoc = goal
        if len(markers)>0:
            for i in range(0,len(markers)):
                m = markers[i]
                # check to see if there are any vicon markers located within 2 cm of the old point
                mrange  = norm(m-goal)
                #stillThere = m[0] == goal[0] and m[1] == goal[1]
                stillThere = norm < 0.05
                #print 'makers',stillThere,m,len(markers)
                #time.sleep(1)
                if stillThere:
                    updatedLoc = m
                    break
        return stillThere,updatedLoc


    def pickUpStuff(self):
        # always make sure that we have stopped
        pose = self.pose_handler.getPose()
        self.drive_handler.setVelocity(0,0,pose[2])
        time.sleep(2)
        self.actuator.pickup(True)

    def cleanUp(self):
        # always make sure that we have stopped
        pose = self.pose_handler.getPose()
        self.drive_handler.setVelocity(0,0,pose[2])
        time.sleep(2)
        self.actuator.drop(True)


    def turnInPlace(self,goal):
        # TODOs: finish this turning
        pose = self.pose_handler.getPose()
        curr_angle = pose[2]
        goal_angleFromOrigin =  math.atan2(goal[1]-pose[1],goal[0]-pose[0])
        angle2Turn = goal_angleFromOrigin - curr_angle- 22*math.pi/180 # offset by 22 degrees just for now
        while angle2Turn <= -math.pi or angle2Turn >= math.pi:

            if angle2Turn <= -math.pi:
                angle2Turn += 2*math.pi
            elif angle2Turn >= math.pi:
                angle2Turn -= 2*math.pi
        while abs(angle2Turn) > 0.05:
            # cap the turning angle between -180 and 180
            print 'POSE:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(pose)))
            print 'marker:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(goal)))

            degrees = int(round(angle2Turn*180/math.pi))
            print 'angle to turn: ',degrees,pose[2],goal_angleFromOrigin
            if abs(degrees)>2:
                if degrees < 0:
                    radiusMM = -1
                else:
                    radiusMM = 1
                data_angle = pack('>BhhBhBhhB',137,30,radiusMM,157,degrees,137,0,0,154)
                response = self.iRobotCommunicator.broadcastToCreate(data_angle,True)
            time.sleep(2) # wait for a bit otherwise turning operation cannot complete
            #print 'ready to turn again!',response

            pose = self.pose_handler.getPose()
            curr_angle = pose[2]
            goal_angleFromOrigin =  math.atan2(goal[1]-pose[1],goal[0]-pose[0])
            angle2Turn = goal_angleFromOrigin - curr_angle-22*math.pi/180

            while angle2Turn <= -math.pi or angle2Turn >= math.pi:
                if angle2Turn <= -math.pi:
                    angle2Turn += 2*math.pi
                elif angle2Turn >= math.pi:
                    angle2Turn -= 2*math.pi
