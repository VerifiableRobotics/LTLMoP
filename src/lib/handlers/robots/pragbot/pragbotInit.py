"""Provides initialization for other Gumbo handlers."""

import xmlrpclib

class gumboInitHandler:

    NODE_NAME = "gumbo_controller"
    PRAGBOT_LISTEN_PORT = 20003

    def __init__(self, proj):  # pylint: disable=W0613
        self.pragbot_proxy = xmlrpclib.ServerProxy(
            "http://localhost:{}".format(self.PRAGBOT_LISTEN_PORT), allow_none=True)

    def getSharedData(self):
        """Return a dict of objects shared with other handlers."""
        return {self.pragbot_proxy}

    def _close(self):
        """Shut down the ROS node."""
        pass
