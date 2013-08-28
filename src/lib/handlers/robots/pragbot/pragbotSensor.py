"""LTLMoP sensor handler for the JR platform."""

import re
from threading import RLock

import numpy
import logging


class sensorHandler(object):
    """Report the robot's current sensor status."""

    def __init__(self, proj, shared_data):  # pylint: disable=W0613

        self._name = type(self).__name__

        # Store reference to proj and pose handler
        self._pose_handler = proj.h_instance['pose']
        self._proj = proj

        # Store reference to server proxy
        self._proxy = \
        proj.h_instance['init'][proj.currentConfig.main_robot].getSharedData()["proxy"]

        # Initialize lock, current objects sensed, and list of objects to ignore
        self._id_fiducials = dict()
        self._disabled_items = set()
        self._currently_sensed = set()
        self._fake_sensed = set()
        self._sensor_lock = RLock()

        # Keep track of region we were in when last polled
        self._last_region = None

    def _set_sensors(self, msg):
        """Reads current sensor status from the robot."""
        pass

    def get_sensor(self, sensor_name, initial=False):
        """Report whether we currently see a fiducial of the requested type.

        sensor_name (string): The type of the fiducial to query.
        """
        if initial:
            # Set the last region to wherever we are now
            self._last_region = self._pose_handler.get_location()
            return True
        else:
            current_region = self._pose_handler.get_location()
            with self._sensor_lock:
                return self._proxy.receiveHandlerMessages(sensor_name, current_region)

    def disable_item(self, item):
        pass

    def get_sensed_item(self, name):
        """Return a fiducial matching a name, None if there is no match."""
        with self._sensor_lock:
            return self._proxy.receiveHandlerMessages("Find Bomb", name)

    def _get_all_sensors(self):
        """Return all things currently sensed.

        To be used for testing only.
        """
        pass

    def set_action_done(self, action_name, value):
        """Set whether an action is done to the given value."""
        pass

