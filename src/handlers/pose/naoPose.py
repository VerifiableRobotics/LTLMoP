#!/usr/bin/env python
"""
=======================================
naoPose.py - Nao Pose Handler
=======================================

Get data from Vicon.
"""

from numpy import *
from socket import *
from struct import *
from threading import Thread, Lock, Event
import sys
import time

global listener

class poseHandler:
    def __init__(self, proj, shared_data):
        try:
            # Get vicon connection settings from robot configuration file
            ViconPort = int(proj.robot_data['ViconPort'][0])    # Vicon listener port (number)
            # Note that this number must be the same as is used when running ViconBroadcaster.exe
        except KeyError, ValueError:
            print "(POSE) ERROR: Cannot find Vicon connection setting ('ViconPort') in robot file."
            exit(-1)
        
        startListening(ViconPort)
        pose = getPoseVicon()       # Clear the blank first return
        time.sleep(0.5)             # Try to make sure only first return is blank
    
    def getPose(self,cached=False):
        # Find out position and orientation from Vicon
        pose = getPoseVicon()
        
        i = 0
        totpose = [0,0,0]
        while i < 5:
            while len(pose) < 6:        # If Vicon outputs empty array (cannot see robot?)
                print "(POSE) ERROR: Vicon pose information unavailable."
                time.sleep(0.2)
                pose = getPoseVicon()   # Try again
            totpose[0] = totpose[0]+pose[0]
            totpose[1] = totpose[1]+pose[1]
            totpose[2] = totpose[2]+pose[3]
            i = i+1
        
        pos_x = totpose[0]/5
        pos_y = totpose[1]/5
        theta = totpose[2]/5
        
        return array([pos_x, pos_y, theta])

class ListeningThread(Thread):
    def __init__(self, port):
        super(ListeningThread, self).__init__()
        self.port = port
        self.poseStr = ""
        self.lock = Lock()
        self._stop = Event()
        
    def run (self):
        # Set the socket parameters
        host = "localhost"
        bufsize = 48
        addr = (host, self.port)

        # Create socket and bind to address
        UDPSock = socket(AF_INET,SOCK_DGRAM)
        UDPSock.bind(addr)
        
        # Receive messages
        while 1:
            if self._stop.isSet():
                break
            data,addr = UDPSock.recvfrom(bufsize)
            if not data:
                print "Client has exited!"
                break
            else:
                self.lock.acquire()
                self.poseStr = data
                self.lock.release()
        # Close socket
        UDPSock.close()
        
    def getPose(self):
        data = ()
        if len(self.poseStr) == 48:
            self.lock.acquire()
            data = unpack('dddddd', self.poseStr)
            self.lock.release()
        return data
        
    def stop (self):
        self._stop.set()
    
    def stopped (self):
        return self._stop.isSet()

def startListening(port):
    global listener
    listener = ListeningThread(port)
    listener.start()

def stopListening():
    global listener
    listener.stop()

# Returns pose as tuple (x, y, z, yaw, pitch, roll)
def getPoseVicon():
    global listener
    if listener != None:
        return listener.getPose()
    else: 
        return ()

if __name__ == '__main__':
    # Test the actuator handler
    ViconPort = 9560
    shared_data = {'ViconPort':ViconPort}
    pose_handler = poseHandler(None, shared_data)
    
    i = 0
    while i < 10:
        pose = pose_handler.getPose()
        print pose
        time.sleep(1)
        i = i+1