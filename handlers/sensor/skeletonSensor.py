#!/usr/bin/env python
"""
===========================================
skeletonSensor.py - Skeleton Sensor Handler
===========================================
"""

class sensorHandler:
    def __init__(self, proj, shared_data):
        # TODO: Initialize any network connections that might be necessary.
        #
        # This is only called once at the beginning of execution.
        #
        # Hostnames/port numbers can be loaded as proj.robot_data['XXXX_PORT'][0]
        # if you stick them in the robot configuration file
        #
        # Note that if the same network socket needs to be shared with another handlers,
        # you should initialize it in an initializationHandler and stick a reference
        # to the socket in shared_data.

        pass

    def getSensorValue(self, sensor_name):
        # TODO: Return a boolean value corresponding to the state of the sensor
        # with name ``sensor_name``
        #
        # If such a sensor does not exist, return ``None``

        return None


