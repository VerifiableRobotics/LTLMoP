#!/usr/bin/env python
"""
Reads from Vicon via Orca
"""

import sys, time
from numpy import *

class poseHandler:
    def __init__(self, shared_data):
        try:
            self.sock = shared_data['Pos2DSock']
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

        while (time.time()-time_stamp)>0.01:
            data, sender = self.sock.recvfrom(1500)
            #print "Packet size: " + str(len(data))
            packet_doubles = array.array('d')
            packet_doubles.fromstring(data)
            time_stamp = packet_doubles[0] + (1e-6)*packet_doubles[1]

        pos_x = packet_doubles[2]
        pos_y = packet_doubles[3]
        pos_o = packet_doubles[4]
        #print "X: " + str(pos_x)
        #print "Y: " + str(pos_y)
        #print "Orientation: " + str(pos_o)

        sys.stderr = olderr

        self.lastPose = array([pos_x, pos_y, pos_o])

        return self.lastPose
    
    def getLastPose(self):
        return self.lastPose

