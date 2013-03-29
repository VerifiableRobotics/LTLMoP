#!/usr/bin/env python

class nullInitHandler:
    def __init__(self, proj):
	pass

    def getSharedData(self):
        # Return a dictionary of any objects that will need to be shared with
        # other handlers
        return {}

