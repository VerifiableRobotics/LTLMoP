#!/usr/bin/env python
"""
=====================================================
handlers/sensor/dummySensor.py - Dummy Sensor Handler
=====================================================

Displays a silly little window for faking sensor values by clicking on buttons.
"""

import threading, subprocess, os, time

class sensorHandler:
    def __init__(self, proj, shared_data):
        """
        Start up sensor handler subwindow and create a new thread to listen to it.
        """
        
        # Since we don't want to have to poll the subwindow for each request,
        # we need a data structure to cache sensor states:
        self.sensorValue = {}

        self.sensorListenInitialized = False
        
        # Create a subprocess
        print "(SENS) Starting sensorHandler window and listen thread..."
        self.p_sensorHandler = subprocess.Popen(os.path.join(proj.ltlmop_root,"handlers","sensor","SensorHandler.py"), stderr=subprocess.PIPE, stdin=subprocess.PIPE)

        self.fd_sensorHandler = self.p_sensorHandler.stderr

        # Create new thread to communicate with subwindow
        sensorListenThread = threading.Thread(target = self.sensorListen)
        sensorListenThread.start()

        # Block until the sensor listener gets the go-ahead from the subwindow
        while not self.sensorListenInitialized:
            time.sleep(0.05) # Yield cpu

        # Tell the subwindow what buttons to create/enable
        for sensor in proj.all_sensors:
            if sensor in proj.initial_sensors:
                self.p_sensorHandler.stdin.write(sensor + ",1\n")
            else:
                self.p_sensorHandler.stdin.write(sensor + ",0\n")

        # Initialize our internal cache
        for sensor in proj.all_sensors:
            self.sensorValue[sensor] = (sensor in proj.initial_sensors)

    def getSensorValue(self, sensor_name):
        """
        Return a boolean value corresponding to the state of the sensor with name ``sensor_name``

        If such a sensor does not exist, returns ``None``
        """

        if sensor_name in self.sensorValue:
            return self.sensorValue[sensor_name]
        else:
            print "(SENS) WARNING: Sensor %s is unknown!" % sensor_name
            return None

    def sensorListen(self):
        """
        Processes messages from the sensor handler subwindow, and updates our cache appropriately
        """

        while 1: 
            # Wait for and receive a message from the subwindow
            input = self.fd_sensorHandler.readline()
            if input == '':  # EOF indicates that the connection has been destroyed
                print "(SENS) Sensor handler listen thread is shutting down."
                break

            # Check for the initialization signal, if necessary
            if not self.sensorListenInitialized and input.strip() == "Hello!":
                self.sensorListenInitialized = True
                continue

            # Get the data out of the message
            args = input.strip().split("=")

            if len(args) != 2:
                continue
            
            # Update our internal cache
            self.sensorValue[args[0]] = (args[1] == "True")

