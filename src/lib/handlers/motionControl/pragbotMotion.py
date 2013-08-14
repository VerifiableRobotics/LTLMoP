"""LTLMoP motion handler for the JR platform."""

from pragbotMotionController import MotionController

class motionControlHandler(object):
    """Send drive commands using MotionController."""

    def __init__(self, proj, shared_data):  # pylint: disable=W0613
        """
        init_node (bool): create separate ROS node (default: False)
        """

        self._name = type(self).__name__
        self._next_region = None
        self._regions = proj.rfi.regions
        self._handlers = proj.h_instance

    def gotoRegion(self, current_region, next_region):
        """Try to drive to next_region, return whether we have arrived."""
        # Look up regions
        # TODO: Add error checking
        current_region = self._regions[current_region].name
        next_region = self._regions[next_region].name

        if current_region == next_region:
            # Stop the robot, which will also clear the next region
            self.stop()
            return True
        elif self._next_region == next_region:
            # We're already trying to go there. Check whether we've arrived.
            if self._at_destination():
                self._next_region = None
                print "{}: Arrived at destination {!r}.".format(
                    self._name, next_region)
                return True
            else:
                # This line should be commented for debugging only, as it will
                # print at the refresh interval of LTLMoP, currently 20Hz
                # print "{}: Continuing move from {!r} to {!r}.".format(
                #    self._name, current_region, next_region)
                return False
        else:
            # Double check that we're not already there.
            if self._at_destination():
                # This should only occur in the rare case that the
                # robot has changed regions since FSA last checked.
                print "{}: Already at destination {!r}.".format(
                    self._name, next_region)
                return True
            else:
                print "{}: Moving from {!r} to {!r}.".format(
                    self._name, current_region, next_region)
                self._next_region = next_region
                print
                return False

    def stop(self):
        """Stop motion."""
        # Controller's stop will print a message if we are actually stopping.
        self._next_region = None

    def _at_destination(self):
        """Return whether we have reached our destination."""
        return self._get_location() == self._next_region

    def _get_location(self):
        """Return the current region."""
        return self._handlers['pose'].get_location()
