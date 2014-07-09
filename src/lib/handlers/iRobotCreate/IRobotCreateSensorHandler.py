#!/usr/bin/env python
"""
=====================================
iRobotCreateSensor.py - Sensor Handler for iRobotCreate
=====================================
"""

import threading, subprocess, os, time, socket
from struct import pack,unpack
from numpy import *
from scipy.linalg import norm

import lib.handlers.handlerTemplates as handlerTemplates

class IRobotCreateSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):

        self.iRobotCommunicator = shared_data['robocomm']
        self.pose_handler = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler)
        self.hsub= executor.hsub

        self.rfi = executor.proj.rfi

    ###################################
    ### Available sensor functions: ###
    ###################################
    def bump_left(self,initial=False):
        """
        returns the current state of the left bumper sensor on the iCreate
        """
        if initial:
            return True
        else:
            query = pack('>BB',142,7)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot![bump_left]'
                return False
            else:

                temp = unpack('>B',data[0])
                return bool(temp[0]&0x0002)

    def bump_right(self,initial=False):
        """
        returns the current state of the right bumper sensor on the iCreate
        """
        if initial:
            return True
        else:
            query = pack('>BB',142,7)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot![bump_right]'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0]&0x0001)

    def wheelDrop_left(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,7)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0]&0x0004)

    def wheelDrop_right(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,7)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0]&0x0003)

    def wheelDrop_caster(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,7)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0]&0x0005)

    def wall(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,8)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0])

    def virtual_wall(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,13,initial=False)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0])

    def cliffFront_left(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,10)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0])

    def cliffFront_right(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,11)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0])

    def cliff_right(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,12)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0])

    def cliff_left(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,9)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0])

    def button_play(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,18)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0]&0x0001)

    def button_advance(self,initial=False):
        if initial:
            return True
        else:
            query = pack('>BB',142,18)
            data = self.iRobotCommunicator.broadcastToCreate(query,True)
            if data is None:
                print '(SENSOR ERROR): Sensor handler failed to received anything response from the robot!'
                return False
            else:
                temp = unpack('>B',data[0])
                return bool(temp[0]&0x0004)

    def nearViconMarkers(self,detection_range=0.8,initial=False):
        """
        Uses Vicon to get all markers on the field that are currently not attached
        to any subjuct and withing the specified detection range of the robot

        detection_range (float): The desired detection range of markers from the center of the robot (default=0.8)
        """
        if initial:
            # start Vicon marker stream, make sure the C# streaming is on!
            self.iRobotCommunicator.runViconMarkerStream()
            return True
        else:
            # get robot's current pose
            robotPose = self.pose_handler.getPose()
            regions = self.rfi.regions
            # get all the marker locations through Vicon
            rawStream = self.iRobotCommunicator.getViconMarkers()
            markers = self._getMarkersNearby(robotPose,regions,rawStream,detection_range)
            # update the communicator so actuator can use this information

            self.iRobotCommunicator.updateViconMarkers(markers)
            target = self.iRobotCommunicator.getTargetMarker()
            if target is not None:
                print 'marker:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(target)))
            elif bool(len(markers)>0) and target is None:
                print 'marker:%d,%d' % tuple(map(int, self.hsub.coordmap_lab2map(markers[0])))
            return bool(len(markers)>0)

    def arrivedAtMarker(self,initial=False):
        if initial:
            return True
        else:
            arrived = self.iRobotCommunicator.hasArrived()
            # reset the arrived flag for next iteration
            if arrived:
                self.iRobotCommunicator.setHasArrived(False)
            return arrived

    def _getMarkersNearby(self,pose,regions,raw,threshold):
        markers = []
        if len(raw) > 0:
            #print 'I see some markers in the raw vicon stream!'
            # first find current region
            curr_region = 0
            map_pose = self.hsub.coordmap_lab2map(pose[0:2])
            for i in range(0,len(regions)):
                if regions[i].objectContainsPoint(map_pose[0],map_pose[1]):
                    curr_region = i
                    break
            #print 'time to see if any of them lie in the region: ',curr_region

            for i in range(0,len(raw)):
                p = raw[i]
                map_p = self.hsub.coordmap_lab2map([p[0],p[1]])

                #dist = abs(norm(array(raw[i])-array([pose[0],pose[1]])))

                if regions[curr_region].objectContainsPoint(map_p[0],map_p[1]):
                    dist = abs(norm(array(raw[i])-array(pose[0:2])))
                    #print 'Found a marker in the current region! Distance: ',dist
                    if dist>0.33:
                        markers.append(raw[i])



        return markers
