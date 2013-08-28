"""Provides initialization for other Gumbo handlers."""

import xmlrpclib

class pragbotInitHandler:

    def __init__(self, proj):  # pylint: disable=W0613
        handler_port = proj.executor.additional_args['handler_port']
        self.pragbot_proxy = xmlrpclib.ServerProxy(
            "http://localhost:{}".format(handler_port), allow_none=True)

    def getSharedData(self):
        """Return a dict of objects shared with other handlers."""
        return {"proxy":self.pragbot_proxy}

    def _close(self):
        """Shut down the ROS node."""
        pass
