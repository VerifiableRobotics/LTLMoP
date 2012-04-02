#!/usr/bin/env python
"""
================================================================
basicSimInit.py -- Basic Simulated Robot Initialization Handler
================================================================
"""
import simulator.basic.basicSimulator as basicSimulator

class initHandler:
    def __init__(self, proj, init_region):
        """
        Initialization handler for pioneer ode simulated robot.

        init_region (region): The name of the region where the simulated robot starts
        """

        # Start in the center of the defined initial region
        for i,r in enumerate(proj.rfiold.regions):
            if r.name == init_region:
                init_region = r
                break
        init_region = proj.rfi.regions[proj.rfi.indexOfRegionWithName(proj.regionMapping[init_region.name][0])]
        center = init_region.getCenter()
        #initialize the simulator
        self.simulator =  basicSimulator.basicSimulator([center[0],center[1],0.0])

    def getSharedData(self):
        # Return a dictionary of any objects that will need to be shared with
        # other handlers
        return {'BasicSimulator':self.simulator}

