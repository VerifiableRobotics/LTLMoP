"""
================================================================================
HexapodActuatorHandler.py - The Hexapod's Actuator Handler
================================================================================
"""

import time

import lib.handlers.handlerTemplates as handlerTemplates

class HexapodActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):
        """
        Acutator handler for hexapod
        """
        
        # get serial port of hexapod
        try:
            self.hexapodSer = shared_data["hexapodSer"]
        except:
            print "(ACTUATOR) ERROR: No connection to  hexapod"
            exit(-1)

    def _sendCommand(self, cmd):
        """
        Send locomotion command ``cmd`` to the robot
        """

        self.hexapodSer.write(cmd)
        x = self.hexapodSer.read()
        while x != 'q':
            print "received invalid ack: ", repr(x)
            self.hexapodSer.write(cmd)
            x = self.hexapodSer.read()
        self.hexapodSer.flush()
       
        time.sleep(0.1)
    
    def standUp(self, actuatorVal, initial=False):
        """
        tells robot to go up
        """
        if initial:
            return
        if actuatorVal:
            self._sendCommand('e')
        else:
            self._sendCommand('b')

    def sitDown(self, actuatorVal, initial=False):
        """
        tells robot to go down
        """
        if initial:
            return
        if actuatorVal:
            self._sendCommand('f')
        else:
            self._sendCommand('b')

    def pincers(self, actuatorVal, initial=False):
        """
        open or close pincers
        
        open of actuatorVal = true
        close if actuatorVal = false
        """
        if initial:
            return
        if actuatorVal:
            self._sendCommand('h')
        else:
            self._sendCommand('g')
        
    def nod(self, actuatorVal, initial=False):
        """
        nods head
        """
        if initial:
            return
        if actuatorVal:
            self._sendCommand('i')
        else:
            self._sendCommand('b')
            
    def shake(self, actuatorVal, initial=False):
        """
        shakes head
        """
        if initial:
            return
        if actuatorVal:
            self._sendCommand('j')
        else:
            self._sendCommand('b')
