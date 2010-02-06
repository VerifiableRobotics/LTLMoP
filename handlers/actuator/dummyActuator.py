#!/usr/bin/env python
"""
Does nothing more than print the actuator name and state; for testing purposes.
"""

import time

class actuatorHandler:
    def __init__(self, shared_data):
        pass
    def setActuator(self, name, val):
        if val:
            time.sleep(0.5)  # Fake some time lag for enable
        print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, val)))

