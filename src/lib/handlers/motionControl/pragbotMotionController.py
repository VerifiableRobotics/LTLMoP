"""Controller for LTLMoP motion handler for the JR platform."""


class MotionController(object):
    """Send movement messages to the robot controller."""


    def __init__(self, proxy):
        self._proxy = proxy

    def stop(self):
        """Send message to pragbot_client to stop robot"""
        self._proxy.receiveHandlerMessages("Stop")

    def go_region(self, region):
        """Send message to pragbot_client to send robot to region"""
        self._proxy.receiveHandlerMessages("Move", region)
