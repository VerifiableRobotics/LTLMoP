"""
================================================================================
HexapodLocomotionCommandHandler.py - The Hexapod's Locomotion Command Handler
================================================================================
"""

import logging
import globalConfig

import lib.handlers.handlerTemplates as handlerTemplates

class HexapodLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    def __init__(self, executor, shared_data):
        """
        Locomotion handler for Hexapod
        """

        # get serial port of hexapod
        try:
            self.hexapodSer = shared_data["hexapodSer"]
        except:
            logging.exception("Couldn't connect to Hexapod")
            exit(-1)

    def sendCommand(self, cmd):
        """
        Send locomotion command ``cmd`` to the robot
        """

        self.hexapodSer.write(cmd)

        x = self.hexapodSer.read()
        while x != 'q':
            logging.debug("received invalid ack: {}".format(x))
            self.hexapodSer.write(cmd)
            x = self.hexapodSer.read()

    def move(self, distance):
        """
        tells robot to move forward
        """
        self.sendCommand('a')

    def stop(self):
        """
        tells robot to stop
        """
        self.sendCommand('b')

    def turnClockwise(self):
        """
        tells robot to turn
        """
        self.sendCommand('c')

    def turnCounterclockwise(self):
        """
        tells robot to turn
        """
        self.sendCommand('d')


