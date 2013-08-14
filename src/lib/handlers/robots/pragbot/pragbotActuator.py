"""LTLMoP motion handler for the JR platform."""
import sys
import time

# Time it takes to defuse a bomb, in seconds
DEFUSE_TIME = 5.0
# Wall distance for sweep
SWEEP_WALL_DISTANCE = 1.3


class gumboActuatorHandler(object):
    """Send actuation commands to the robot."""

    def __init__(self, proj, shared_data):  # pylint: disable=W0613
        self._name = type(self).__name__
        self._sensor_handler = proj.h_instance['sensor'][proj.currentConfig.main_robot]
        self._pose_handler = proj.h_instance['pose']

        # Sweep
        self._sweep_client = None
        self._sweep_goal = None

        # Defuse
        self._defuse_client = None
        self._defuse_goal = None

        self.executor = proj.executor

    def sweep(self, actuatorVal, initial=False):
        """Perform a search of the current room."""
        # Create action client
        if initial:
            print "Sweep Area Action"
            return True

        # Activate or deactivate sweep
        actuatorVal = _normalize(actuatorVal)
        if actuatorVal:
            print "{}: Activating sweep.".format(self._name)
            print "Searching the {}...".format(self._pose_handler.get_location())
            self._sweep_goal.timeout = 0.0  # Never timeout
            self._sweep_goal.wall_dist = SWEEP_WALL_DISTANCE

            # Set up a lexically enclosed callback for setting the _done sensor
            def _complete_sweep(goal_status, goal_result):  # pylint: disable=W0613
                """Set sweep_done sensor when sweep is complete."""
                if goal_status == True:
                    print "{}: Sweep succeeded.".format(self._name)
                    print "Search Complete"
                    self._sensor_handler.set_action_done("sweep", True)
                elif goal_status == False:
                    print "{}: Sweep cancelled before completion.".format(self._name)
                else:
                    print "{}: Sweep failed with status {!r}.".format(self._name, goal_status)

            self._sweep_client.send_goal(self._sweep_goal, done_cb=_complete_sweep)
            return True
        else:
            print "{}: Deactivating sweep.".format(self._name)
            if self._sweep_goal:
                self._sweep_client.cancel_goal()
                self._sweep_goal = None

            # Reset the sweep_done sensor
            self._sensor_handler.set_action_done("sweep", False)
            return True

    def defuse(self, actuatorVal, initial=False):
        """Defuse a bomb by driving to it and making it disappear."""
        # Create action client
        if initial:
            print "Drive to..."
            return True

        # Activate or deactivate defuse
        actuatorVal = _normalize(actuatorVal)
        if actuatorVal:
            # Get the bomb from the sensors
            bomb = self._sensor_handler.get_sensed_item("bomb")
            if not bomb:
                print "{}: Defuse requested but no bomb found.".format(self._name)
                return False

            # Move the robot to the bomb
            bomb_position = bomb.pose.position

            self.executor.postEvent("MESSAGE", "Defusing...")

            # Sneakily define a lexically enclosed callback
            def _complete_defuse(goal_status, goal_result):  # pylint: disable=W0613
                """Wait until bomb is defused and then remove it from the sensors."""
                if goal_status == True:
                    # After the robot reaches the bomb, defuse the
                    # bomb by waiting and then make it go away.
                    print "{}: Defusing bomb...".format(self._name)
                    time.sleep(DEFUSE_TIME)
                    print "{}: Bomb defusing complete.".format(self._name)
                    self.executor.postEvent("MESSAGE", "Successfully defused.")
                    self._sensor_handler.disable_item(bomb)
                elif goal_status == False:
                    print "{}: Defuse cancelled before completion.".format(self._name)
                else:
                    print "{}: Defuse failed with status {!r}.".format(self._name, goal_status)

            self._defuse_client.send_goal(self._defuse_goal, done_cb=_complete_defuse)
            print "{}: Moving to bomb at ({}, {}).".format(self._name, bomb_position.x,
                                                          bomb_position.y)
            return True
        else:
            print "{}: Deactivating defuse.".format(self._name)
            if self._defuse_goal:
                self._defuse_client.cancel_goal()
                self._defuse_goal = None
            return True


def _normalize(value):
    """Normalize the value that an actuator is being set to."""
    # If it's not a boolean, it will be a string of an int.
    if not isinstance(value, bool):
        return bool(int(value))
    else:
        return value
