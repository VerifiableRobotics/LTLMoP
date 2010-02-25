#!/usr/bin/env python
"""
Sensor handler for color ball detection
"""


from socket import *
import sys, struct, time, os
import array as pyarray


class sensorHandler:
    def __init__(self, proj, shared_data):
        """
        Opens socket, subscribes to multicast group, and calculates 
        local vs. Vicon clock offset (to ensure most recent data).

        Hostnames and ports are read in from the robot description file.
        """

        ### Connect to Orca:

        try:
            self.COLOR_GROUP = proj.robot_data['ColorDetectionGroup'][0]
            self.COLOR_PORT = int(proj.robot_data['ColorDetectionPort'][0])
        except KeyError, ValueError:
            print "(SENSOR) ERROR: Cannot find Orca network settings ('ColorDetectionGroup', 'ColorDetectionPort') in robot description file."
            sys.exit(-1)

        # Open up sockets
        print '(SENSOR) Subscribing to Orca multicast stream...'
        self.color_sock = socket(AF_INET, SOCK_DGRAM)
        self.color_sock.bind(('', self.COLOR_PORT))

        # Join group
        group_bin = inet_pton(AF_INET, self.COLOR_GROUP)
        mreq = group_bin + struct.pack('=I', INADDR_ANY)
        self.color_sock.setsockopt(IPPROTO_IP, IP_ADD_MEMBERSHIP, mreq)

        # Calculate clock offset
        data = self.color_sock.recv(1500)
        now = time.time()
        packet_doubles = pyarray.array('d')
        packet_doubles.fromstring(data)
        time_stamp = packet_doubles[0] + (1e-6)*packet_doubles[1]
        self.time_offset = time_stamp - now
        
        print "(SENSOR) Detected time delay of %fsec." % self.time_offset
        print "(SENSOR) OK! We've successfully connected."

        self.last_update = 0
        self.red_values = [0]*6 #Initialize off
        self.red_index = 0 # Index for ring buffer

    def readFromOrca(self):
        MIN_DELAY = 0.01  # seconds

        #TODO: It would be nice if we could actually just flush the socket...

        now = time.time() + self.time_offset
        time_stamp = 0.0
        while (now-time_stamp)>MIN_DELAY:
            data = self.color_sock.recv(1500)
            #print "Packet size: " + str(len(data))
            packet_doubles = pyarray.array('d')
            packet_doubles.fromstring(data)
            time_stamp = packet_doubles[0] + (1e-6)*packet_doubles[1]
        
        value = packet_doubles[2]
        #print "t = " + str(time_stamp) + ", detecting " + str(value)

        return value

            
    def getSensorValue(self, sensor_name):
        """
        Return a boolean value corresponding to the state of the sensor with name ``sensor_name``

        If such a sensor does not exist, returns ``None``
        """

        MIN_BLOB_PERIOD = 0.1

        if sensor_name in ['fire', 'hazardous_item']:
            return False
        elif sensor_name == 'person':
            now = time.time()
            if (now - self.last_update) > 0.1:
                orca_val = self.readFromOrca()
                self.red_values[self.red_index] = (orca_val > 0.5 and orca_val < 1.5)

                self.red_index += 1
                if self.red_index == len(self.red_values):
                    self.red_index = 0

                self.last_update = now
    
            red_visible = sum(self.red_values) > 1
            return red_visible
        else:
            print "WARNING: Sensor %s is unknown!" % sensor_name
            return None



 
