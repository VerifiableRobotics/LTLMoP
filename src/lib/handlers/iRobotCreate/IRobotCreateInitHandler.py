#!/usr/bin/env python
"""
=================================================
iRobotCreateInit.py - iRobotCreate Initialization Handler
=================================================

Initialize the sockets to communicate with iRobotCreate(beagleboard) on the Pioneer
"""

import time, math
from socket import *
from struct import pack,unpack
from threading import Thread, Lock, Event
import threading, subprocess, os
from numpy import matrix


import lib.handlers.handlerTemplates as handlerTemplates

class IRobotCreateInitHandler(handlerTemplates.InitHandler):
    def __init__(self,executor,listenIP = "0.0.0.0",broadCastIP = "192.168.1.120",createPort=8865,beaglePort=8866,artagPort=8844,sonarPort=8833,buffer=1024):
        """
        Initialization handler for iRobotCreate robot

        listenIP (string): the ip address of the device that is currently running LTLMoP (default="0.0.0.0")
        broadCastIP (string): The ip address of the iRobotCreate. Not sure? Log on to the Create router and checkout the ip address! (default="192.168.1.120")
        createPort (int): BeagleBoard's port for transferring commands directly to iCreate (default=8865)
        beaglePort (int): BeagleBoard's control port (default=8866)
        artagPort (int): BeagleBoard's port for streaming artags (default=8844)
        sonarPort (int): BeagleBoard's port for streaming sonar data (default=8833)
        buffer (int): specifies the size of the buffer for the listener (default=1024)
        """

        try:
            # Create proxies to access modules
            self.robocomm = _iRobotCreateCommunicator(executor.proj,listenIP,broadCastIP,createPort,beaglePort,artagPort,sonarPort,buffer)
            self.robocomm.start()
            time.sleep(1)   # Give communicator time to start and receive first data
        except RuntimeError:
            self.robocomm.stop()
            print "(INIT) ERROR: Cannot connect to the robot."
            exit(-1)

    def getSharedData(self):
        # Return dictionary of module proxies for other handlers to use
        return {'robocomm':self.robocomm}

########################################################################
###                Main _iRobotCreateCommunicator Class               ###
########################################################################

class _iRobotCreateCommunicator:

    # Constructor
    def __init__(self,proj,listenIP,broadCastIP,createPort,beaglePort,artagPort,sonarPort,buffer):

        # Communication parameters
        self.listenerIP = listenIP
        self.createPort = createPort
        self.beaglePort = beaglePort
        self.artagPort = artagPort
        self.sonarPort = sonarPort
        self.broadcastIP = broadCastIP
        self.buffer = buffer

        # Communication parameters
        self.beagleAddr = (self.broadcastIP,self.beaglePort)
        self.createAddr = (self.broadcastIP,self.createPort)
        self.beagleSock = socket(AF_INET,SOCK_STREAM)
        self.createSock = socket(AF_INET,SOCK_STREAM)
        # Communication threads
        self.listener = _RobotListener(self.listenerIP,self.artagPort,self.buffer)
        self.broadcaster = _RobotBroadcaster(proj,self.broadcastIP,self.beagleSock,self.createSock,self.beagleAddr,self.createAddr,self.buffer)
        self.viconMarkerListener =  _ViconMarkerListener(20.0,"0.0.0.0",7500)# just use the default

        self.targetMarker = None
        self.viconMarkers = []
        self.arrived = False

        self.pickUpAttempt = 0

    def start(self):
        """
        Open sockets for communication.
         """
        # test connection to the beagle board/Create, similar to CreateBeagleInit.m
        self.broadcaster.testRun()
        # starts the listener thread
        self.listener.start()
        time.sleep(1)



    def stop(self):
        """
        Close sockets and prevent further communication.
        """
        self.listener.stop()
        self.broadcaster.stop()
        self.viconMarkerListener.stop()

    def sendDirection(self,direction):
        """
        Serialize and send the direction vector command to the robot.
        Also used to send velocity commands to the robot.

        direction (tuple containing 2 doubles):(x,y) or (v,w) depending on application.
        """
        self.broadcaster.sendDirection(direction)
    def broadcastToCreate(self,data,response=False):
        """
        Sends the data stream to the create

        data (packed byte strings): contains the serial data stream that you would like to send
        """
        if response:
            received = self.broadcaster.sendCommand(data,response)
            return received
        else:
            self.broadcaster.sendCommand(data,response)

    def broadcastToBeagle(self,data):
        self.broadcaster.sendToBeagle(data)

    def runViconMarkerStream(self):
        if not self.viconMarkerListener.isAlive():
            self.viconMarkerListener.start()

    def getViconMarkers(self):
        return self.viconMarkerListener.getViconMarkers()

    def setHasArrived(self,state):
        self.arrived = state

    def hasArrived(self):
        self.targetMarker = None
        return self.arrived


    def updateViconMarkers(self,markers):
        self.viconMarkers = markers

    def lockOnTargetMarker(self):
        if len(self.viconMarkers)>0:
            self.targetMarker = self.viconMarkers[0]
            return self.targetMarker
        else:
            self.targetMarker = None
            return None
    def getTargetMarker(self):
        return self.targetMarker

    def updatePickUpAttempt(self):
        self.pickUpAttempt += 1





########################################################################
###            _RobotListener Class for Processing ARTags             ###
########################################################################

class _RobotListener(Thread):

    def __init__(self,Host,ARTagSock,buff):
        super(_RobotListener,self).__init__()
        self.ARTagAddr = (Host,ARTagSock)
        self.ARTagUDPSock=socket(AF_INET,SOCK_DGRAM)

        self.ARTagUDPSock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.ARTagUDPSock.bind(self.ARTagAddr)
        self.tagsReceived = {}
        self.buf = buff
        self.lock = Lock()
        self.close = Event()

    def run(self):

       print '(IROBOTCOMM)starting _RobotListener!'
       count = 0
       while not self.close.isSet():
           data,addr = self.ARTagUDPSock.recvfrom(self.buf)
           self.lock.acquire()
           self.processARTag(data)
           self.lock.release()
           time.sleep(0.05)
       print '(IROBOTCOMM)Exiting Listener!'

    def stop(self):
        self.close.set()

    def processARTag(self,tags):
        self.tagsReceived = {} # clear data from last time
        length = len(tags)
        MAXARTAGSEEN = 10 # maximum number of ARtags that will get detected in 1 camera view
        HEADER = 12

        if length>15:
            for i in range(0,MAXARTAGSEEN-1):
                idIndex = HEADER+(i)*4
                ID = unpack('<i',tags[idIndex:idIndex+4])
                ID = ID[0]
                #print [tags[idIndex],tags[idIndex+1],tags[idIndex+2],tags[idIndex+3],tags[idIndex+4],tags[idIndex+5],tags[idIndex+6],tags[idIndex+7],tags[idIndex+8],tags[idIndex+9]]
                if ID == -1:
                    break
                xIndex = idIndex+4*(MAXARTAGSEEN)
                #print [tags[xIndex-2],tags[xIndex-1],tags[xIndex],tags[xIndex+1],tags[xIndex+2],tags[xIndex+3],tags[xIndex+4],tags[xIndex+5],tags[xIndex+6],tags[xIndex+7],tags[xIndex+8]]
                x = unpack('<f',tags[xIndex:xIndex+4])
                x = x[0]
                yIndex = xIndex+4*MAXARTAGSEEN
                y = unpack('<f',tags[yIndex:yIndex+4])
                y=y[0]
                zIndex = yIndex+4*MAXARTAGSEEN
                z = unpack('<f',tags[zIndex:zIndex+4])
                z = z[0]
                yawIndex = zIndex+4*MAXARTAGSEEN
                yaw = unpack('<f',tags[yawIndex:yawIndex+4])
                yaw = yaw[0]
                self.tagsReceived[i] = [ID,x,y,z,yaw]
                #print 'OOOO I FOUND SOME AR TAGS!!!',len(tags)
            if len(self.tagsReceived)>0:
                tag1 = self.tagsReceived[0]
                print "ARTAG: ID = %s x = %s y = %s z = %s yaw = %s" % (str(tag1[0]), str(tag1[1]), str(tag1[2]), str(tag1[3]),str(tag1[4]))



class _RobotBroadcaster:
    """
    Class used to communicate from LTLMoP to a robot.
    """

    # Constructor
    def __init__(self,proj,ipAddress,beagleSock,createSock,beagleAddr,createAddr,bufferSize):
        # Communication parameters
        self.createAddr = createAddr
        self.beagleAddr = beagleAddr
        self.beagleSock = beagleSock
        self.createSock = createSock
        self.proj =proj

    # Disable communication
    def stop(self):
        """
        Close the socket to end UDP communication.
        """
        print 'iRobot: I am shutting down!'
        self.beagleSock.send(pack('B',8))
        time.sleep(1)
        self.beagleSock.close()
        self.createSock.close()

    def testRun(self):
        """
        Performs a test run on the connection to Beagle Board similar
        to the MATLAB program CreateBeagleInit(lights up LEDs etc)
        """
        print 'iRobot: Opening connection to iRobot Create...'
        #print 'beagle port:',self.beagleAddr,self.createAddr
        self.beagleSock.connect(self.beagleAddr)
        time.sleep(1)
        self.beagleSock.send(pack('B',1))              # open connection to beagle board first
        time.sleep(2)
        self.createSock.connect(self.createAddr)
        self.createSock.send(pack('B',128))            # see if the robot is alive
        time.sleep(0.5)
        self.createSock.send(pack('B',132))            # set create in full control mode
        time.sleep(0.5)
        self.createSock.send(pack('BBBB',139,25,0,128))# light LEDs
        self.createSock.send(pack('BBBBBBBBB',140,1,3,48,20,52,20,55,20))# set song
        data = pack('BBBBBBB',140,2,2,48,20,52,20) # this is used later for actuation

        time.sleep(0.5)
        self.createSock.send(pack('BB',141,1))         # sing it!!

        print 'iRobot: I am alive if I just sang a song :D'
        time.sleep(1) # why do I need to sleep again?


    def sendDirection(self,direction):
        """
        Serialize and send the direction vector command to the robot.
        Also used to send velocity commands to the robot.

        direction is a tuple containing 2 doubles.
        (x,y)
        or (v,w) depending on application.
        """
        x,y = direction
        wheelVel = matrix([[1,0.129],[1,-0.129]])*matrix([[x],[y]])

        rightWheelVel = min(max(500*wheelVel[0,0],-500),500)
        leftWheelVel = min(max(500*wheelVel[1,0],-500),500)

        data = pack('>Bhh',145,rightWheelVel,leftWheelVel)
        self.createSock.sendto(data,self.createAddr)

    def sendCommand(self,data,response=False):
        self.createSock.sendto(data,self.createAddr)
        if response:
            received = self.createSock.recv(1024)
            while len(received)<1:
                received += self.createSock.recv(1024)
            return received

    def sendToBeagle(self,data):
        self.beagleSock.sendto(data,self.beagleAddr)

class _ViconMarkerListener(Thread):
    def __init__(self, freq=20.0, ip="0.0.0.0", port=7500):
        """Create the a socket to receive Vicon data.

        freq - Update frequency in Hz.
        ip - IP address to listen on.
             Default is local computer.
        port - Port to listen on.
               Default matches ViconMarkerBroadcaster (C#) default
        parent - Object that may be useful. Can change ProcessData method to
                 make it do something with this object when data is received.
        """
        super(_ViconMarkerListener, self).__init__()

        # Communication parameters
        self.updateFreq = freq
        self.addr = (ip, port)
        self.bufsize = 65536
        self.udpSock = socket(AF_INET, SOCK_DGRAM)
        self.lock = threading.Lock()
        self.close = threading.Event()

        # Container for marker positions
        self.poses = []         # List of tuples [(x1, y1), (x2, y2), ...]

    def run(self):
        """Open the socket to start communication. Process messages."""
        # Open socket for communication
        self.udpSock.bind(self.addr)
        print 'Starting Vicon Streamer! '
        # Receive communication until stopped
        self.close.clear()
        delay = 1.0 / self.updateFreq
        while not self.close.isSet():

            self.lock.acquire()
            data = self.udpSock.recv(self.bufsize)
            self.lock.release()
            self.ProcessData(data)
            time.sleep(delay)

        # Close socket
        self.udpSock.close()

    def stop(self):
        """Close the socket to end UDP communication."""
        self.close.set()

    # Deserialize and save data
    def ProcessData(self, data):
        """Extract marker positions and keep them.

        data - Byte array encoded from multiple pairs of doubles [x1 y1 ...]
        """
        temp_poses = []
        # Check for valid data (not null or incomplete)
        if data and len(data)%16 == 0:
            self.poses = []
            for i in range(0, len(data), 16):
                x, y = unpack('dd', data[i:i+16])
                temp_poses.append((x, y))
            # If you want something to happen every time poses are recieved
            # put that in here. You may need to pass in parameter parent.
        self.poses = temp_poses

    def getViconMarkers(self):
        #print 'poses sent to sensor! ', len(self.poses)
        return self.poses

