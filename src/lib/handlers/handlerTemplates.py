#!/usr/bin/env python


""" ================================================
    handlerTemplates.py Defines templates for handler classes.  
    All handlers should be subclasses of an appropriate handler type (e.g. InitHandler)
    ================================================
"""

import globalConfig

class Handler(object):
    def __init__(self, executor, *args, **kwds):
        super(Handler, self).__init__(*args, **kwds)
        self.executor = executor

    def _stop(self):
        """
        Properly terminates all threads/computations in the handler. Leave no trace behind.
        """
        logging.debug("WARNING: No _stop() function implemented, may cause problems on shutdown")
    
    def _onProjectUpdated(self, newPrject):
        """
        Get called when the current project is updated
        """
        pass

    def _onMapUpdated(self, newMap):
        """
        Get called when the current map is updated
        """
        pass

class InitHandler(Handler):
    """
    Perform any pre-experiment initialization
    """
    def __init__(self, *args, **kwds):
        super(InitHandler, self).__init__(*args, **kwds)

class PoseHandler(Handler):
    """
    Handle connection to pose provider and abstraction (continuous pose -> region in discrete workspace)
    """
    def __init__(self, *args, **kwds):
        super(PoseHandler, self).__init__(*args, **kwds)

    def getPose(self):
        raise NotImplementedError

class SensorHandler(Handler):
    """
    Handle connection to sensors and abstraction (continuous sensor value -> discretized value over proposition(s))
    """
    def __init__(self, *args, **kwds):
        super(SensorHandler, self).__init__(*args, **kwds)

class ActuatorHandler(Handler):
    """
    Handle connection to actuators and abstraction (discretized actuator value -> continuous action)
    """
    def __init__(self, *args, **kwds):
        super(ActuatorHandler, self).__init__(*args, **kwds)

class MotionControlHandler(Handler):
    """
    Control high-level region-to-region movement (start region, dest. region -> motion)
    Needs to specify whether it depends on lower-level movement handlers or not (or both)
    Needs to provide a stop() method to abort any action and stay in place
    """
    def __init__(self, *args, **kwds):
        super(MotionControlHandler, self).__init__(*args, **kwds)
    
    def 

class DriveHandler(Handler):
    """
    Converts global [v_x, v_y] vector to-- robot local velocity for example-- [v, w]
    Needs to specify what type(s) of command it generates
    """
    def __init__(self, *args, **kwds):
        super(DriveHandler, self).__init__(*args, **kwds)

class LocomotionCommandHandler(Handler):
    """
    Sends motor commands to robot
    Needs to specify what type(s) of command it expects to receive
    """
    def __init__(self, *args, **kwds):
        super(LocomotionCommandHandler, self).__init__(*args, **kwds)
