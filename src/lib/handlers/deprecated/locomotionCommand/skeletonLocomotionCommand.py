#!/usr/bin/env python
"""
==================================================================
skeletonLocomotionCommand.py - Skeleton Locomotion Command Handler
==================================================================
"""

class locomotionCommandHandler:
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

    def sendCommand(self, cmd):
        # TODO: Send locomotion command ``cmd`` to the robot
        pass

