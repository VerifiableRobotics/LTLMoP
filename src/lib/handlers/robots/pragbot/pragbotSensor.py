"""LTLMoP sensor handler for pragbot."""

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


class sensorHandler(object):
    """Report the robot's current sensor status."""

    def __init__(self, proj, shared_data):  # pylint: disable=W0613

        self._name = type(self).__name__

        # Store reference to proj
        self._proj = proj

        # Store reference to server proxy
        self._proxy = \
        proj.h_instance['init'][proj.currentConfig.main_robot].getSharedData()["proxy"]


    def get_sensor(self, sensor_name, initial=False):
        """Report whether we currently see a fiducial of the requested type.

        sensor_name (string): The type of the fiducial to query.
        """
        if initial:
            # Nothing to do on initialization
            return True
        else:
            # TODO: Implement this to match what the pragbot system
            # expects. No idea whether this line reflects the current
            # implementation. Commented out for now. Another line used
            # self._proxy.receiveHandlerMessages("Find Bomb",
            # sensor_name). Take a look at the pragbot game code to
            # figure out what it should be.
            # return self._proxy.receiveHandlerMessages(sensor_name, current_region)
            return False

    def set_action_done(self, action_name, value):
        """Set whether an action is done to the given value."""
        # TODO: Implement
        raise NotImplementedError()
