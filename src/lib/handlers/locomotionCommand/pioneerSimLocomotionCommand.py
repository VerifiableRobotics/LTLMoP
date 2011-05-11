#!/usr/bin/env python
# -*- coding: cp1252 -*-
"""
================================================================================
DiffDriveSimLocomotionCommand.py - Pioneer Simulation Locomotion Command Handler
================================================================================
"""
import socket, sys, time


class locomotionCommandHandler:
    def __init__(self, proj, shared_data):
        pass

    def sendCommand(self, cmd):

        # Command the robot based on the gait given by the drive handler.
		
		v = 12.0*cmd[0]
		w = 12.0*cmd[1]
                time.sleep(0.5)
		HOST, PORT = "localhost", 23456
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
               
                vel = '('+ str(v) +","+ str(w) +')'
                sock.sendto("LTLMOP%" + vel + "\n", (HOST, PORT))
                received = sock.recv(1024)
               
