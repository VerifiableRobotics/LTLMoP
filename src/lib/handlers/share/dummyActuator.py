#!/usr/bin/env python
"""
=========================================
dummyActuator.py - Dummy Actuator Handler
=========================================

Does nothing more than print the actuator name and state; for testing purposes.
"""

import time

class actuatorHandler:
    def __init__(self, proj, shared_data):
        pass

    def setActuator(self, name, val):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (bool).
        
        name (string): Name of the actuator
        val (bool): the state of the actuator
        """

        if val:
            time.sleep(0.1)  # Fake some time lag for the actuator to enable

        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

