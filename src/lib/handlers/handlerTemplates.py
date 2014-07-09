#!/usr/bin/env python


""" ================================================
    handlerTemplates.py Defines templates for handler classes
    All handlers should be subclasses of an appropriate handler type (e.g. InitHandler)
    ================================================
"""
import re
import logging
from collections import OrderedDict
import globalConfig

class Handler(object):
    """
    A generic handler object
    """
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
        """ Return the position of the robot """
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

class LoadingError(Exception):
    """An exception when any handler fails to load """
    def __init__(self, msg):
        # Call the base class constructor with the parameters it needs
        Exception.__init__(self, msg)

# a dictionary that maps handler type in str to handler type class object
handler_type_mapping = OrderedDict([('Init', InitHandler), \
                        ('Pose', PoseHandler), \
                        ('LocomotionCommand', LocomotionCommandHandler), \
                        ('Drive', DriveHandler), \
                        ('MotionControl', MotionControlHandler), \
                        ('Sensor', SensorHandler), \
                        ('Actuator', ActuatorHandler)
                       ])

# update the dictionary to include bidirectional mapping
handler_type_mapping.update(OrderedDict([(v, k) for k, v in handler_type_mapping.iteritems()]))

def getHandlerTypeClass(name):
    """
    Given a handler type name in string, return the corresponding handler class object
    """
    if not isinstance(name, str):
        raise TypeError("Expected handler type name as string")

    # if the name ends with `Handler`, remove it
    name = re.sub('Handler$', '', name)

    try:
        return handler_type_mapping[name]
    except KeyError:
        raise KeyError("Invalid handler type")

def getHandlerTypeName(h_class, short_name = True):
    """
    Given a handler class object, return the corresponding handler type name in string
    If short_name is True, return the name without tailing `Handler`
    """
    if Handler not in h_class.__bases__:
        raise TypeError("Expected handler type as handler class object")

    try:
        return handler_type_mapping[h_class] + ('Handler' if not short_name else '')
    except KeyError:
        raise KeyError("Invalid handler type")

def getAllHandlerTypeName(short_name = True):
    """
    Return a list of handler type name in string
    If short_name is True, return the name without tailing `Handler`
    """
    return [h_type + ('Handler' if not short_name else '') \
            for h_type in handler_type_mapping.keys() if isinstance(h_type, basestring)]

def getAllHandlerTypeClass():
    """
    Return a list of handler type class
    """
    return [h_type for h_type in handler_type_mapping.keys() if not isinstance(h_type, basestring)]

if __name__ == '__main__':
    print getAllHandlerTypeClass()
    print getAllHandlerTypeName()
    pass
