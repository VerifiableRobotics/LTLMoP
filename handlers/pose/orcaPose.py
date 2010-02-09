#!/usr/bin/env python
"""
Reads from Vicon via Orca
"""

import sys, time, struct
import array as pyarray
from socket import *
from numpy import *

class poseHandler:
    def __init__(self, shared_data):
        try:
            self.sock = shared_data['Pos2DSock']
            self.time_offset = shared_data['TimeOffset']
        except KeyError:
            print "(POSE) ERROR: Orca connection doesn't seem to be initialized!"
            sys.exit(-1)
        
        self.lastPose = None # For caching

        # Test
        print "Initial pose: " + str(self.getPose())
            
    def getPose(self):
        time_stamp = 0.0

        #TODO: get the most current data
        #TODO: pos2d_sock.fflush()    

        now = time.time() + self.time_offset
        while (now-time_stamp)>0.01:
            data = self.sock.recv(1500)
            #print "Packet size: " + str(len(data))
            packet_doubles = pyarray.array('d')
            packet_doubles.fromstring(data)
            time_stamp = packet_doubles[0] + (1e-6)*packet_doubles[1]

        pos_x = packet_doubles[2]
        pos_y = packet_doubles[3]
        pos_o = packet_doubles[4]
        #print "X: " + str(pos_x)
        #print "Y: " + str(pos_y)
        #print "Orientation: " + str(pos_o)

        self.lastPose = array([pos_x, pos_y, pos_o])

        return self.lastPose
    
    def getLastPose(self):
        return self.lastPose

