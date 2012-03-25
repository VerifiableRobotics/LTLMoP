#!/usr/bin/env python
"""
=====================================
dummySensor.py - Dummy Sensor Handler
=====================================

Displays a silly little window for faking sensor values by clicking on buttons.
"""

import threading, subprocess, os, time, socket

class sensorHandler:
    def __init__(self, proj, shared_data):
        """
        Start up sensor handler subwindow and create a new thread to listen to it.
        """
        
        # Since we don't want to have to poll the subwindow for each request,
        # we need a data structure to cache sensor states:
        self.sensorValue = {}
        self.proj = proj
        self.sensorListenInitialized = False
        

        # Create a subprocess
        print "(SENS) Starting sensorHandler window and listen thread..."
        self.p_sensorHandler = subprocess.Popen(["python", os.path.join(self.proj.ltlmop_root,"lib","handlers","share","_SensorHandler.py")], stderr=subprocess.PIPE, stdin=subprocess.PIPE)

        self.fd_sensorHandler = self.p_sensorHandler.stderr

        # Create new thread to communicate with subwindow
        sensorListenThread = threading.Thread(target = self._sensorListen)
        sensorListenThread.start()

        # Block until the sensor listener gets the go-ahead from the subwindow
        while not self.sensorListenInitialized:
            time.sleep(0.05) # Yield cpu

    def buttonPress(self,button_name,init_value,initial=False):
        """
        Return a boolean value corresponding to the state of the sensor with name ``sensor_name``
        If such a sensor does not exist, returns ``None``

        button_name (string): Name of the sensor whose state is interested
        init_value (bool): The initial state of the sensor
        """
        if initial:
            if button_name not in self.sensorValue.keys(): 
                self.sensorValue[button_name] = init_value
                if init_value:
                    self.p_sensorHandler.stdin.write(button_name + ",1\n")
                else:
                    self.p_sensorHandler.stdin.write(button_name + ",0\n")
            return self.sensorValue[button_name]
        else:
            if button_name in self.sensorValue:
                return self.sensorValue[button_name]
            else:
                print "(SENS) WARNING: Sensor %s is unknown!" % button_name
                return None


    def _getSensorValue(self, sensor_name, initial=False):
        """
        Return a boolean value corresponding to the state of the sensor with name ``sensor_name``
        If such a sensor does not exist, returns ``None``

        sensor_name (string): Name of the sensor whose state is interested
        """

        if sensor_name in self.sensorValue:
            return self.sensorValue[sensor_name]
        else:
            print "(SENS) WARNING: Sensor %s is unknown!" % sensor_name
            return None

    def _sensorListen(self):
        """
        Processes messages from the sensor handler subwindow, and updates our cache appropriately
        """
        host = 'localhost'
        port = 23459    
        buf = 1024
        addr = (host,port)

        UDPSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        UDPSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        UDPSock.bind(addr)
        
        while 1: 
            # Wait for and receive a message from the subwindow
            input,addrFrom = UDPSock.recvfrom(1024)
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

