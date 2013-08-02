#!/usr/bin/env python
"""
=====================================
CSharpRobotSensor.py - Sensor Handler for CSharpRobots: Pioneer and Segway
=====================================
"""

import threading, subprocess, os, time, socket
from ltlmopMsg_pb2 import *
from struct import pack,unpack
from numpy import *
from scipy.linalg import norm
from regions import Point, Color
import Polygon
import copy
from specCompiler import SpecCompiler
import json
import logging, globalConfig

class sensorHandler:
    def __init__(self, proj, shared_data):

        self.CSharpCommunicator = proj.shared_data['robocomm']
        self.pose_handler = proj.h_instance['pose']
        self.proj = proj

        self.rfi = proj.loadRegionFile()

        # sensor variables
        self.exploring = False
        self.gotNewRegion = False
        self.mapThread = _MapUpdateThread(self.proj)
        self.mapThread.daemon = True
        self.oldExternalFaces = []


    ###################################
    ### Available sensor functions: ###
    ###################################
    def artag_detection(self,ARTagID,initial=False):
        """
        returns the ARTag ID currently observed
        ARTagID (int): the specific ARTag number we are looking for (default=0)
        """
        if initial:
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id=4
            ltlmop_msg.sensor=PythonRequestMsg.ARTAG
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            #print 'got sensor init resp ',response
            return True
        else:
            ARTAG = self.CSharpCommunicator.getARTAG()
            status = False
            if len(ARTAG)>0:
                for x in range(0,len(ARTAG)):
                    if ARTAG[x] == ARTagID:
                        status = True
            if status:
                print 'I see ARTAG:',ARTagID
            return status
    
    def lidar_detection(self,initial=False):
        """
        returns the LIDAR points currently observed
        """
        if initial:
            ltlmop_msg = PythonRequestMsg()
            
            ltlmop_msg.id=3
            
            ltlmop_msg.sensor=PythonRequestMsg.LIDAR
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            print 'got sensor init resp ',response
            return True
        else:

            return False

    def busyExploring(self,initial=False):
        """
        returns true if we just requested an explore actuation to C#
        responsible for checking to see if C# is done exploring and
        setting the appropriate flag in LTLMoP
        """
        if initial:
            return True
        else:
            if self.CSharpCommunicator.getExploreBusy(): #we don't get velocity updates so gotta do this ourselves
                ltlmop_msg = PythonRequestMsg()
                ltlmop_msg.id=5
                ltlmop_msg.sensor = PythonRequestMsg.NOSENSOR
                ltlmop_msg.actuator.actuatorType = PythonRequestMsg.EXPLORE
                ltlmop_msg.actuator.status = PythonRequestMsg.REQ_UPDATE
                msg = self.CSharpCommunicator.sendMessage(ltlmop_msg)
                if (msg.actuator is not None and msg.actuator.actuatorType!=PythonRequestMsg.NOACT):
                    self.BUSY_EXPLORE = msg.actuator.status==PythonRequestMsg.RESP_BUSY

                if(self.BUSY_EXPLORE and not self.exploring):
                    self.exploring = True
                #print 'dont get V',self.BUSY_EXPLORE
                return self.BUSY_EXPLORE
            else:
                #print 'get V so false'
                return False
    
    def doneExploring(self,initial=True):
        """
        returns true if LTLMoP senses that CSharp has just finished exploring
        """
        if initial:
            return True
        else:
            if (self.exploring and (not self.CSharpCommunicator.getExploreBusy() or not self.BUSY_EXPLORE)):
                self.exploring = False
                return True
            elif (self.exploring):
                return False
            else:
                return False
     
    def doorClosed(self, initial = False):
        #TODO: Document the crap out of this function. ~sm2296
        """
        A sensor with two purposes: (1) to get LIDAR amd GridSLAM running on the robot/DEASL/C# end,(2) to let LTLMoP know whether the space in front of the robot is free or not. Not free implies that a door would be closed.
        On the C# side, it uses the method 'openDoorRequest'. (Without this sensor enabled in LTLMoP, the LTLMoP-C# Interface won't enable LIDAR and GridSLAM.)
        """
        
        proj = self.proj
        # initially, it sends the map to C# and returns True.
        if initial:
            # init LIDAR at the CSharp end
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id = 6
            ltlmop_msg.sensor=PythonRequestMsg.OPENDOOR
            # update the proj with the most current rfi
            # this is needed after resynthesis, otherwise the rfi is still the old one
            self.rfi = proj.rfiold
            print [r.name for r in self.rfi.regions]
            # make sure we only send the newer rfi when we have finished exploring the region
            if (self.mapThread.mapType==PythonRequestMsg.NEWREGIONFOUND and len(self.oldExternalFaces)>0):
                self.exposedFaces = self.oldExternalFaces
                print 'old external faces!'
            else:
                self.exposedFaces = self.rfi.getExternalFaces() # a list of faces that are not connected to aother regions in the map
                if self.exposedFaces is None:
                    self.exposedFaces = proj.rfi.getExternalFaces()
                self.oldExternalFaces = self.exposedFaces
            print 'exposedFaces:',len(self.exposedFaces)# see how many external faces we have generated
            # get the points the external faces
            for face_pta, face_ptb in self.exposedFaces:
                new_f = ltlmop_msg.Face()
                face_p1 = self.proj.coordmap_map2lab(face_pta)
                face_p2 = self.proj.coordmap_map2lab(face_ptb)
                new_f.p1.x = face_p1[0]
                new_f.p1.y = face_p1[1]
                new_f.p2.x = face_p2[0]
                new_f.p2.y = face_p2[1]
                ltlmop_msg.exfaces.faces.extend([new_f]) # add face to list
            # get points in the current map
            for r in self.rfi.regions:
                new_r = ltlmop_msg.Region()
                points = map(self.proj.coordmap_map2lab,r.getPoints())
                holes = [map(proj.coordmap_map2lab, r.getPoints(hole_id=i)) for i in xrange(len(r.holeList))]
                #print 'name',r.name,len(r.pointArray)
                print 'region direction',r.name,Polygon.Polygon(r.pointArray).orientation()
                if Polygon.Polygon(r.pointArray).orientation()==1:
                    for p in points:
                        new_p = ltlmop_msg.Point()
                        new_p.x = p[0]
                        new_p.y = p[1]
                        new_r.points.extend([new_p])
                else:
                    for p in reversed(points):
                        new_p = ltlmop_msg.Point()
                        new_p.x = p[0]
                        new_p.y = p[1]
                        new_r.points.extend([new_p])
                for h in holes:
                    new_h = ltlmop_msg.Point()
                    new_h.x = h[0]
                    new_h.y = h[1]
                    new_r.holes.extend([new_h])
                new_r.name = r.name
                print '*****',r.name
                ltlmop_msg.map.r.extend([new_r])# add all the map/external faces to the message
            ltlmop_msg.map.type = PythonRequestMsg.REGIONUPDATE
            
            # start the map update thread
            if (self.mapThread.notStarted):
                self.mapThread.start()
            # setup a sensor in case C# wants to know how we are doing
            sensor = ltlmop_msg.Sensor()
            sensor.type = PythonRequestMsg.OPENDOOR
            ltlmop_msg.sensors.extend([sensor])
            if self.mapThread.processing:
                sensor.stat = 1
            else:
                sensor.stat = 0 # IDLE
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            print "!!!!!!!!!!!! This is the (initial == True) ltlmop_msg:", response
            if len(response.map.r)!=0 and not self.mapThread.processing:
                print "updated MAP RECEIVED",response.map.type
                self.mapThread.updateMap(response.map)      
            print 'LIDAR/GridSLAM were initialized! ~Spyros'
            return True
        # after that, ignores the C# map updates, uses the previous map, and returns True or False, depending on whether a door would be closed or open, respectively.
        else:
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id = 7           # What does this ID mean?
            sensor = ltlmop_msg.Sensor()
            sensor.type = PythonRequestMsg.OPENDOOR
            ltlmop_msg.sensor=PythonRequestMsg.OPENDOOR
            ltlmop_msg.sensors.extend([sensor])
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            #print "!!!!!!!!!!!! This is the first ltlmop_msg:", response
            
            ltlmop_msg.id = 75          # this is for when we don't have a new map to send
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            #print "!!!!!!!!!!!! This is the second ltlmop_msg:", response

            s = response.sensors[0]
            
            if (s.type == PythonRequestMsg.OPENDOOR):
                front_free = s.stat     # -1 for occupied, 0 for unknown, 1 for free. See method 'openDoorRequest' on C# side.
                #print "front_free now is {!r}".format(front_free)
            else:
                print "unexpected message for opendoor response"
                
            return (front_free != 1)    # The C# method returns 1 for 'front of robot is free', but we want 1 for 'door closed'.
    
    def open_doorway(self,initial=False):
        """
        returns whether a new LIDAR has discovered a new doorway that was
        not included in the original map
        """
        proj = self.proj
        if initial:
            # init LIDAR at the CSharp end
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id=6
            ltlmop_msg.sensor=PythonRequestMsg.OPENDOOR
            # update the proj with the most current rfi
            # this is needed after resynthesis, otherwise the rfi is still the old one
            self.rfi = proj.rfiold
            print [r.name for r in self.rfi.regions]
            # make sure we only send the newer rfi when we have finished exploring the region
            if (self.mapThread.mapType==PythonRequestMsg.NEWREGIONFOUND and len(self.oldExternalFaces)>0):
                self.exposedFaces = self.oldExternalFaces
                print 'old external faces!'
            else:
                self.exposedFaces = self.rfi.getExternalFaces() # a list of faces that are not connected to aother regions in the map
                if self.exposedFaces is None:
                    self.exposedFaces = proj.rfi.getExternalFaces()
                self.oldExternalFaces = self.exposedFaces
            print 'exposedFaces:',len(self.exposedFaces)# see how many external faces we have generated
            # get the points the external faces
            for face_pta, face_ptb in self.exposedFaces:
                new_f = ltlmop_msg.Face()
                face_p1 = self.proj.coordmap_map2lab(face_pta)
                face_p2 = self.proj.coordmap_map2lab(face_ptb)
                new_f.p1.x = face_p1[0]
                new_f.p1.y = face_p1[1]
                new_f.p2.x = face_p2[0]
                new_f.p2.y = face_p2[1]
                ltlmop_msg.exfaces.faces.extend([new_f]) # add face to list
            # get points in the current map
            for r in self.rfi.regions:
                new_r = ltlmop_msg.Region()
                points = map(self.proj.coordmap_map2lab,r.getPoints())
                holes = [map(proj.coordmap_map2lab, r.getPoints(hole_id=i)) for i in xrange(len(r.holeList))]
                #print 'name',r.name,len(r.pointArray)
                print 'region direction',r.name,Polygon.Polygon(r.pointArray).orientation()
                if Polygon.Polygon(r.pointArray).orientation()==1:
                    for p in points:
                        new_p = ltlmop_msg.Point()
                        new_p.x = p[0]
                        new_p.y = p[1]
                        new_r.points.extend([new_p])
                else:
                    for p in reversed(points):
                        new_p = ltlmop_msg.Point()
                        new_p.x = p[0]
                        new_p.y = p[1]
                        new_r.points.extend([new_p])
                for h in holes:
                    new_h = ltlmop_msg.Point()
                    new_h.x = h[0]
                    new_h.y = h[1]
                    new_r.holes.extend([new_h])
                new_r.name = r.name
                print '*****',r.name
                ltlmop_msg.map.r.extend([new_r])# add all the map/external faces to the message
            ltlmop_msg.map.type = PythonRequestMsg.REGIONUPDATE
            
            # start the map update thread
            if (self.mapThread.notStarted):
                self.mapThread.start()
            # setup a sensor in case C# wants to know how we are doing
            sensor = ltlmop_msg.Sensor()
            sensor.type = PythonRequestMsg.OPENDOOR
            ltlmop_msg.sensors.extend([sensor])
            if self.mapThread.processing:
                sensor.stat = 1
            else:
                sensor.stat = 0 # IDLE
            response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
            print "!!!!!!!!!!!! This is the 'response' variable:", response
            if len(response.map.r)!=0 and not self.mapThread.processing:
                print "updated MAP RECEIVED",response.map.type
                self.mapThread.updateMap(response.map)

                
            print 'got sensor init resp '#we are able to receive a response!
            return True
        else:
            ltlmop_msg = PythonRequestMsg()
            ltlmop_msg.id=7
            sensor = ltlmop_msg.Sensor()
            sensor.type = PythonRequestMsg.OPENDOOR
            if self.mapThread.processing:
                sensor.stat = 1
            else:
                sensor.stat = 0 # IDLE
            ltlmop_msg.sensor=PythonRequestMsg.OPENDOOR
            ltlmop_msg.sensors.extend([sensor])
            if self.mapThread.mapReady:
                ltlmop_msg.id=77
                # for now add the new region to ltlmsg and update C#
                # temporary regions should not be grown out of temporary regions
                    # external faces unchanged until we are sure that his is the right map
                
                if (self.mapThread.mapType==PythonRequestMsg.NEWREGIONFOUND):
                    self.exposedFaces = self.oldExternalFaces
                else:
                    self.exposedFaces = self.mapThread.rfi.getExternalFaces() # a list of faces that are not connected to aother regions in the map
                    #if (len(self.exposedFaces)==0):
                    #    self.mapThread.rfi.regions.pop(self.mapThread.rfi.indexOfRegionWithName("boundary"))
                    #    self.exposedFaces = self.mapThread.rfi.getExternalFaces() # a list of faces that are not connected to aother regions in the map
                    
                    print 'NOT NEW REGION ONLY AN UPDATE!',len(self.proj.rfi.regions)#,len(self.mapThread.rfi)]
                print 'exposedFacesB:',len(self.exposedFaces)
                # get the most updated external face from the handler so we can let C# know
                for face_pta, face_ptb in self.exposedFaces:
                    new_f = ltlmop_msg.Face()
                    face_p1 = self.proj.coordmap_map2lab(face_pta)
                    face_p2 = self.proj.coordmap_map2lab(face_ptb)
                    new_f.p1.x = face_p1[0]
                    new_f.p1.y = face_p1[1]
                    new_f.p2.x = face_p2[0]
                    new_f.p2.y = face_p2[1]
                    ltlmop_msg.exfaces.faces.extend([new_f]) # add face to list
                # get the most updated list of map points and let C# know something has changed
                for r in self.mapThread.rfi.regions:
                    new_r = ltlmop_msg.Region()
                    new_r.name = r.name
                    points = map(self.proj.coordmap_map2lab,r.getPoints())
                    holes = [map(proj.coordmap_map2lab, r.getPoints(hole_id=i)) for i in xrange(len(r.holeList))]
                    #print 'region direction',r.name,Polygon.Polygon(r.pointArray).orientation()
                    if Polygon.Polygon(r.pointArray).orientation()==1:
                        for p in points:
                            new_p = ltlmop_msg.Point()
                            new_p.x = p[0]
                            new_p.y = p[1]
                            new_r.points.extend([new_p])
                    else:
                        for p in reversed(points):
                            new_p = ltlmop_msg.Point()
                            new_p.x = p[0]
                            new_p.y = p[1]
                            new_r.points.extend([new_p])

                    for h in holes:
                        new_h = ltlmop_msg.Point()
                        new_h.x = h[0]
                        new_h.y = h[1]
                        new_r.holes.extend([new_h])
                    # lets see what have we got here 
                    print 'name: ',r.name
                    ltlmop_msg.map.r.extend([new_r])
                ltlmop_msg.map.type = PythonRequestMsg.REGIONUPDATE
                response = self.CSharpCommunicator.sendMessage(ltlmop_msg)#off we go!
                print 'lalalala',response.map.type
                self.mapThread.mapReady = False #now we are ready to get a new map!
                # need to make sure that we only trigger when C# has found temporary region and
                # not just finishing up exploration
                if (self.mapThread.mapType==PythonRequestMsg.NEWREGIONFOUND):
                    return True
                else:
                    return False
            else:
                ltlmop_msg.id=75 # this is for when we don't have a new map to send
                response = self.CSharpCommunicator.sendMessage(ltlmop_msg)
                # check to see if we are getting a map from C#
                if len(response.map.r)!=0 and not self.mapThread.processing:
                    # if so, let's keep the mapThread busy
                    print "NEW MAP RECEIVED",response.map.type
                    self.mapThread.updateMap(response.map)

                return False

    def regionAdded(self, initial):
        """ Return true if a region was added in the last map update """

        if initial:
            return
        if self.mapThread.regionAddedFlag.isSet():
            self.addedRegions = copy.deepcopy(self.mapThread.addedRegions)
            self.mapThread.regionAddedFlag.clear()
            return True
        else:
            return False

    def regionRemoved(self, initial):
        """ Return true if a region was removed in the last map update """

        if initial:
            return

        if self.mapThread.regionRemovedFlag.isSet():
            self.removedRegions = copy.deepcopy(self.mapThread.removedRegions)
            self.mapThread.regionRemovedFlag.clear()
            return True
        else:
            return False

#this is a happy thread that will convert a pythonRequestMsg's map field into a region file that
#LTLMoP can interpret! Thread is initiated at the start of open_doorway and map procesing is triggered
#when the gotNewMap flag is set and we are not in the process of processing somthing else
class _MapUpdateThread(threading.Thread):
    def __init__(self, proj, *args, **kwds):
        self.map_basename = proj.getFilenamePrefix().split('.')[0]
        self.project_root = proj.project_root
        self.regionList = set([r.name for r in proj.rfiold.regions])
        self.regionAddedFlag = threading.Event()
        self.regionRemovedFlag = threading.Event()
        self.proj = proj
        self.coordmap_lab2map = proj.coordmap_lab2map
        self.gotNewMap = False
        self.new_map = []
        self.rfi = proj.rfi
        self.mapReady = False;
        self.mapType = 1;
        self.processing = False;
        self.notStarted = True

        super(_MapUpdateThread, self).__init__(*args, **kwds)
    def run(self):
        import regions
        print "running..."
        self.notStarted = False
        map_number = 1
        data = []
        while True:
            if self.gotNewMap and not self.mapReady:
                self.processing = True
                print "GOTnEWmAP!@"
                self.rfi = regions.RegionFileInterface(transitions=[])

                self.rfi.regions = []
                # first convert the map from PythonRequestMsg to a list of regions
                for reg in self.new_map:
                    # transform the points to map coordinates
                    # convert each region
                    r = regions.Region()
                    r.type = 1
                    r.pointArray = []
                    r.name = reg.name
                    count = 0
                    #print 'regions in new map:',len(reg.points)
                    print 'regions in new map name:',r.name,len(reg.points)
                    for p in reg.points: # add all the points of the region in lab coordinates
                        transformed_p = self.coordmap_lab2map(regions.Point(p.x,p.y))
                        r_pos = r.position
                        r.addPoint(regions.Point(round(transformed_p[0]-r_pos.x),round(transformed_p[1]-r_pos.y)),count)
                        count = count + 1
                        print 'addedpoints',p.x,p.y
                    if (len(reg.points)>0):
                        print 'added2RFI:',r.name
                        if (r.name!='boundary'):
                            self.rfi.regions.append(r)
                    
                # recompute the boundary
                self.rfi.doMakeBoundary()
                print 'domakeBoundary done!!!!!!!!!!'

                # When we receive one, write it to a new regions file 
                reg_filename = "%s.update%d.regions" % (self.map_basename, map_number)

                self.rfi.writeFile(reg_filename)

                print "Wrote region file %s." % reg_filename

                # Tell simGUI to update
                print "REGIONS:" + reg_filename

                #self.regionList = set([r.name for r in proj.rfiold.regions])
                newRegionList = set([r.name for r in self.rfi.regions])

                # Check for any substantive changes
                self.addedRegions = newRegionList - self.regionList
                self.removedRegions = self.regionList - newRegionList

                
                map_number += 1
                self.gotNewMap = False
                print "added: " + str(self.addedRegions)
                self.mapReady = True
                if self.addedRegions:
                    self.regionList = newRegionList # make sure we update the reference
                    print "added: " + str(self.addedRegions)
                    self.regionAddedFlag.set()
                self.processing = False


    def updateMap(self, new_map):
        # store the updated map received from PythonRequestMsg
        print "updated map!"
        self.new_map = new_map.r
        self.mapType = new_map.type
        self.gotNewMap = True
        
        
        
