"""
LTLMoP motion handler for the pragbot game.

"""

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

import sys
import time

# Time it takes to defuse a bomb, in seconds
DEFUSE_TIME = 5.0


class gumboActuatorHandler(object):
    """Send actuation commands to the robot."""

    def __init__(self, proj, shared_data):  # pylint: disable=W0613
        self._name = type(self).__name__
        self._sensor_handler = proj.h_instance['sensor'][proj.currentConfig.main_robot]
        self._pose_handler = proj.h_instance['pose']
        self.executor = proj.executor

    def defuse(self, actuatorVal, initial=False):
        """Defuse a bomb by driving to it and making it disappear."""
        # Perform any initialization required
        if initial:
            return True

        # Activate or deactivate defuse
        actuatorVal = _normalize(actuatorVal)
        if actuatorVal:
            # Update the status that we're defusing.
            self.executor.postEvent("MESSAGE", "Defuse activated")
            # TODO: Implement
            # Get the position of the bomb in the room
            
            # If there's no bomb in the room, print an error and
            # return False.  The return value of this function is not
            # checked by fsa.py at the moment, so the return value is
            # never read, but we use it anyway.

            # Update the status that we've defusing.
            self.executor.postEvent("MESSAGE", "Defusing...")

            # Issue a non-blocking command to move the robot to the
            # bomb position.
            print "DEFUSAL IS NOW TAKING OVER MOTION CONTROL"            
            print "Going for bomb."

            # Define up a lexically-enclosed function that will set the
            # 'defuse_done' sensor  and make the bomb disappear.
            def _complete_defuse():  # pylint: disable=W0613
                """Set defuse_done and remove the bomb from the sensors."""
                # TODO: Implement. You want to make it disappear in
                # pragbot itself, and then the sensor handler should
                # get then end up showing no bomb because the game
                # environment changed. To set defuse_done, use the
                # sensor hander's set_action_done.

            # Use a Timer object to call it after DEFUSE_TIME seconds.

            return True
        else:
            # Update the status that we're no longer defusing.
            self.executor.postEvent("MESSAGE", "Defuse deactivated")
            return True


def _normalize(value):
    """Normalize the value that an actuator is being set to."""
    # If it's not a boolean, it will be a string of an int.
    if not isinstance(value, bool):
        return bool(int(value))
    else:
        return value
