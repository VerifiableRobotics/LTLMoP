#!/usr/bin/env python
"""
===============================================
skeletonActuator.py - Skeleton Actuator Handler
===============================================
"""

import time

class actuatorHandler:
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

    def setActuator(self, name, val):
        # TODO: Set actuator of name ``name`` to be in state ``val`` (binary).

        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

