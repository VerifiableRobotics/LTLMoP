"""LTLMoP pose handler for the JR platform."""

from threading import Lock

import numpy


class poseHandler:
    """Report the robot's current pose."""
    def __init__(self, proj, shared_data):  # pylint: disable=W0613
        """
        init_node (bool): create separate ROS node (default: False)
        """

        self._name = type(self).__name__

        # Initialize pose and lock
        self._pose = numpy.array([0, 0, 0])
        self._pose_lock = Lock()

        # Initialize location and lock
        self._location = "Nowhere"
        self._location_lock = Lock()

    def getPose(self, cached=True):  # pylint: disable=W0613
        """Return the last reported pose.

        The optional second argument is required by LTLMoP but we
        ignore it.
        """
        with self._pose_lock:
            return self._pose

    def getRegion(self):
        """Return the last reported region.

        This is a convenience wrapper to match LTLMoP naming conventions.
        """
        return self.get_location()

    def get_location(self):
        """Return the room the robot is currently in."""
        with self._location_lock:
            return self._location

    def set_location(self, location):
        """Set the location of the robot."""
        with self._location_lock:
            self._location = location
