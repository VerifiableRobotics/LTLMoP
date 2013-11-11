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
        
        # Store reference to server proxy
        self._proxy = \
        proj.h_instance['init'][proj.currentConfig.main_robot].getSharedData()["proxy"]

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

            #Set defusing to true for the actuators to lock out the stop command from LTLMoP
            self._proxy.receiveHandlerMessages("Defusing","True")
            location_string = self._proxy.receiveHandlerMessages("Object_Location","bombs")
            jr_location = self._proxy.receiveHandlerMessages("Location","coordinates")
            locations = []
            if not location_string:
                return False
            else:
                pieces = location_string.split(",")
                numbombs = int(pieces[0])
                if numbombs > 0: 
                    locations = pieces[1:-1]
                    locations = [(int(w),int(v)) for w, v in [l.split(":") for l in locations]]
                    location = locations[0]
                else:
                    return False
            
            nearest_face = self._nearest_face(jr_location,location)
            # Update the status via the executor that we're defusing .
            self.executor.postEvent("MESSAGE", "Defusing...")

            # Issue a non-blocking command to move the robot to the
            # bomb position.
            print "DEFUSAL IS NOW TAKING OVER MOTION CONTROL"            
            print "Going for bomb."
            self._proxy.receiveHandlerMessages("Move_Location",nearest_face)
            # Define up a lexically-enclosed function that will set the
            # 'defuse_done' sensor  and make the bomb disappear.
            def _complete_defuse():  # pylint: disable=W0613
                """Set defuse_done and remove the bomb from the sensors."""
                # TODO: Implement. You want to make it disappear in
                # pragbot itself, and then the sensor handler should
                # get then end up showing no bomb because the game
                # environment changed. To set defuse_done, use the
                # sensor hander's set_action_done.                
                self._proxy.receiveHandlerMessages("Defuse",location)
                self._proxy.receiveHandlerMessages("Defusing","False")

            # Use a Timer object to call it after DEFUSE_TIME seconds.
            time.sleep(DEFUSE_TIME)
            _complete_defuse()
            return True
        else:
            # Update the status that we're no longer defusing.
            self.executor.postEvent("MESSAGE", "Defuse deactivated")            
            return True
        
    def _nearest_face(self,here,there):
        """Given two coordinates, return the coordinates of the 
            nearest face of the second coordinates to the first
        """
        self.executor.postEvent("MESSAGE", "Nearest face here:"+str(here)+" there: "+str(there))
        n = here[1] > there[1]
        e = here[0] > there[0]
        s = here[1] < there[1]
        w = here[0] < there[0]
        if n: return (there[0],there[1]+1)
        if e: return (there[0]+1,there[1])
        if s: return (there[0],there[1]-1)
        if w: return (there[0]-1, there[1])
            

def _normalize(value):
    """Normalize the value that an actuator is being set to."""
    # If it's not a boolean, it will be a string of an int.
    if not isinstance(value, bool):
        return bool(int(value))
    else:
        return value
