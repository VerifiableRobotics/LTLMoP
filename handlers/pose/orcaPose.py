#!/usr/bin/env python
"""
==============================
orcaPose.py - Orca Pose Client
==============================

Reads Vicon pose data via Orca
"""

import sys, time, struct
import array as pyarray  # So as not to conflict with numpy's array
from socket import *
from numpy import *

class poseHandler:
    def __init__(self, proj, shared_data):
        """
        Opens socket, subscribes to multicast group, and calculates 
        local vs. Vicon clock offset (to ensure most recent data).

        Hostnames and ports are read in from the robot description file.
        """

        ### Connect to Orca:

        try:
            self.POS2D_GROUP = proj.robot_data['OrcaPositionGroup'][0]
            self.POS2D_PORT = int(proj.robot_data['OrcaPositionPort'][0])
        except KeyError, ValueError:
            print "(POSE) ERROR: Cannot find Orca network settings ('OrcaPositionGroup', 'OrcaPositionPort') in robot description file."
            sys.exit(-1)

        # Open up sockets
        print '(POSE) Subscribing to Orca multicast stream...'
        self.pos2d_sock = socket(AF_INET, SOCK_DGRAM)
        self.pos2d_sock.bind(('', self.POS2D_PORT))

        # Join group
        group_bin = inet_pton(AF_INET, self.POS2D_GROUP)
        mreq = group_bin + struct.pack('=I', INADDR_ANY)
        self.pos2d_sock.setsockopt(IPPROTO_IP, IP_ADD_MEMBERSHIP, mreq)

        # Calculate clock offset
        data = self.pos2d_sock.recv(1500)
        now = time.time()
        packet_doubles = pyarray.array('d')
        packet_doubles.fromstring(data)
        time_stamp = packet_doubles[0] + (1e-6)*packet_doubles[1]
        self.time_offset = time_stamp - now
        
        print "(POSE) Detected time delay of %fsec." % self.time_offset
        print "(POSE) OK! We've successfully connected."

    def getPose(self):
        """
        Gets the most recent pose reading from the multicast stream.
        """

        #TODO: It would be nice if we could actually just flush the socket...

        MIN_DELAY = 0.01  # seconds

        time_stamp = 0.0
        now = time.time() + self.time_offset
        while (now-time_stamp)>MIN_DELAY:
            data = self.sock.recv(1500)
            #print "Packet size: " + str(len(data))
            packet_doubles = pyarray.array('d')
            packet_doubles.fromstring(data)
            time_stamp = packet_doubles[0] + (1e-6)*packet_doubles[1]

        pos_x = packet_doubles[2]
        pos_y = packet_doubles[3]
        pos_o = packet_doubles[4]

        return array([pos_x, pos_y, pos_o])

