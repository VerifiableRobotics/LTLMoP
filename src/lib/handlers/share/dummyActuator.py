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

    def setActuator(self, name, actuatorVal,initial):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (bool).
        
        name (string): Name of the actuator
        """

        if initial:
            # do nothing
            pass
        else:
            if actuatorVal:
                time.sleep(0.1)  # Fake some time lag for the actuator to enable

            print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, actuatorVal)))

