#!/usr/bin/env python
# -*- coding: cp1252 -*-
"""
================================================================================
PioneerODELocomotionCommand.py - Pioneer Simulation Locomotion Command Handler
================================================================================
"""
import socket, sys, time


import lib.handlers.handlerTemplates as handlerTemplates

class PioneerODELocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    def __init__(self, proj, shared_data,speed,host,port):
        """
        LocomotionCommand Handler for pioneer ode robot.

        speed (float): The speed multiplier (default=12.0,min=6.0,max=15.0)
        host (string): The host for communication with simulator, must match the one in UDPServer.py (default="localhost")
        port (int): The port for communication with simulator, must match the one in UDPServer.py (default=23456)
        """
        self.speed = speed
        self.host = host
        self.port = port
        
    def sendCommand(self, cmd):

        v = self.speed*cmd[0]
        w = self.speed*cmd[1]
        time.sleep(0.5)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       
        vel = '('+ str(v) +","+ str(w) +')'
        sock.sendto("LTLMOP%" + vel + "\n", (self.host, self.port))
        received = sock.recv(1024)
               
