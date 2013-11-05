"""LTLMoP pose handler for pragbot."""

# Copyright (C) 2013 Constantine Lignos
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import numpy


class poseHandler:
    """Report the robot's current pose."""
    def __init__(self, proj, shared_data):  # pylint: disable=W0613

        self._name = type(self).__name__

        # Initialize defaul pose
        self._pose = numpy.array([0, 0, 0])

        # Get server proxy from shared data
        self._proxy = \
            proj.h_instance['init'][proj.currentConfig.main_robot].getSharedData()["proxy"]

    def getPose(self, cached=True):  # pylint: disable=W0613
        """Return a default pose.

        Since this handler can only be run in topo-only mode, the pose
        never changes, but region does. getRegion should be called to
        find out where the robot is. The optional second argument is
        required by LTLMoP but we ignore it.

        """
        return self._pose

    def getRegion(self):
        """Return the last reported region.

        This is a convenience wrapper to match LTLMoP naming conventions.
        """
        return self.get_location()

    def get_location(self):
        """Return the region the robot is currently in."""
        return self._proxy.receiveHandlerMessages("Location")
