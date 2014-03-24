#!/usr/bin/env python
"""
================================================================
basicSimInit.py -- Basic Simulated Robot Initialization Handler
================================================================
"""
import lib.simulator.basic.basicSimulator as basicSimulator

import lib.handlers.handlerTemplates as handlerTemplates

class BasicSimInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor, init_region):
        """
        Initialization handler for basic simulated robot.

        init_region (region): The name of the region where the simulated robot starts
        """

        rfi_original = executor.proj.loadRegionFile(decomposed=False)

        # Start in the center of the defined initial region
        init_region_obj = rfi_original.regions[rfi_original.indexOfRegionWithName(init_region)]
        center = init_region_obj.getCenter()

        #initialize the simulator
        self.simulator =  basicSimulator.basicSimulator([center[0],center[1],0.0])

    def getSharedData(self):
        # Return a dictionary of any objects that will need to be shared with
        # other handlers
        return {'BasicSimulator':self.simulator}

