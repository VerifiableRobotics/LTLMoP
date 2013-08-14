"""Provides initialization for other Gumbo handlers."""


class gumboInitHandler:

    NODE_NAME = "gumbo_controller"

    def __init__(self, proj):  # pylint: disable=W0613
        pass

    def getSharedData(self):
        """Return a dict of objects shared with other handlers."""
        return {}

    def _close(self):
        """Shut down the ROS node."""
        pass
