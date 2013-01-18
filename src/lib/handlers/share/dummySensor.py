#!/usr/bin/env python
"""
=====================================
dummySensor.py - Dummy Sensor Handler
=====================================

Displays a silly little window for faking sensor values by clicking on buttons.
"""

import threading, subprocess, os, time, socket
import sys

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
        self._running = True
        self.p_sensorHandler = None

    def _stop(self):
        if self.p_sensorHandler is not None:
            print >>sys.__stderr__, "(SENS) Killing dummysensor GUI..."
            self.p_sensorHandler.stdin.write(":QUIT\n")
            self.p_sensorHandler.stdin.close()

            print >>sys.__stderr__, "(SENS) Terminating dummysensor GUI listen thread..."
            self._running = False
            self.sensorListenThread.join()

    def buttonPress(self,button_name,init_value,initial=False):
        """
        Return a boolean value corresponding to the state of the sensor with name ``sensor_name``
        If such a sensor does not exist, returns ``None``

        button_name (string): Name of the sensor whose state is interested
        init_value (bool): The initial state of the sensor (default=False)
        """
        if initial:

            if not self.sensorListenInitialized:
                # Create a subprocess
                print "(SENS) Starting sensorHandler window and listen thread..."
                self.p_sensorHandler = subprocess.Popen(["python", "-u", os.path.join(self.proj.ltlmop_root,"lib","handlers","share","_SensorHandler.py")], stderr=subprocess.PIPE, stdin=subprocess.PIPE)

                # Create new thread to communicate with subwindow
                self.sensorListenThread = threading.Thread(target = self._sensorListen)
                self.sensorListenThread.daemon = True
                self.sensorListenThread.start()

                # Block until the sensor listener gets the go-ahead from the subwindow
                while not self.sensorListenInitialized:
                    time.sleep(0.05) # Yield cpu

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
        UDPSock.settimeout(1)
        try:
            UDPSock.bind(addr)
        except:
            print "ERROR: Cannot bind to port.  Try killing all Python processes and trying again."
            return

        while self._running:
            # Wait for and receive a message from the subwindow
            try:
                input,addrFrom = UDPSock.recvfrom(1024)
            except socket.timeout:
                continue

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

