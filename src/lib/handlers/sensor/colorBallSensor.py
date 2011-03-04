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
        # Initialize off
        # Index for ring buffer
        self.sensorValues = [{'name': 'person',
                            'values': [0]*6,
                            'index': 0,
                            'range': [0.5, 1.5]},
                            {'name': 'hazardous_item',
                            'values': [0]*6,
                            'index': 0,
                            'range': [1.5, 2.5]},
                            {'name': 'fire',
                            'values': [0]*6,
                            'index': 0,
                            'range': [2.5, 3.5]}]

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
            time_stamp = packet_doubles[1] + (1e-6)*packet_doubles[2]
        
        value = packet_doubles[0]
        #print "t = " + str(time_stamp) + ", detecting " + str(value)

        return value

            
    def getSensorValue(self, sensor_name):
        """
        Return a boolean value corresponding to the state of the sensor with name ``sensor_name``

        If such a sensor does not exist, returns ``None``
        """

        MIN_BLOB_PERIOD = 0.1

        if sensor_name in ['fire', 'person', 'hazardous_item']: # hazard is blue ball, person is red ball, fire is both ball
            now = time.time()
            if (now - self.last_update) > 0.1:
                orca_val = self.readFromOrca()

                for value in self.sensorValues:
                    value['values'][value['index']] = (orca_val > value['range'][0] and orca_val < value['range'][1])
                    value['index'] += 1


                    if value['index'] == len(value['values']):
                        value['index'] = 0

                self.last_update = now 

            for value in self.sensorValues:
                if sensor_name == value['name']:
                    return sum(value['values']) > 1


        else:
            print "WARNING: Sensor %s is unknown!" % sensor_name
            return None



 
