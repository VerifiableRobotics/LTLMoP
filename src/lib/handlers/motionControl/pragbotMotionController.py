"""Controller for LTLMoP motion handler for the JR platform."""

import actionlib
from geometry_msgs.msg import Pose
from robot_actions.msg import DriveToAction, DriveToGoal
from subtle_msgs.srv import GetTopoMap


class MotionController(object):
    """Send movement messages to the robot controller."""

    NODE_NAME = 'motion_controller'

    def __init__(self):
        self._name = type(self).__name__

        # Get a client for driving
        self._drive_client = \
            actionlib.SimpleActionClient('drive_to', DriveToAction)
        self._drive_goal = None


    def stop(self):
        """Attempt to stop motion."""
        # If there's no active goal, there's nothing to be done
        if self._drive_goal:
            self._drive_client.cancel_goal()
            self._drive_goal = None
            print "{}: Cancelled robot drive goal.".format(self._name)

