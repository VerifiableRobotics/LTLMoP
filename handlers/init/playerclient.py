#!/usr/bin/env python
"""
Create a player client and connect to a specified player server
Returns the tuple (player client object, position2d proxy object) 
"""

import textwrap, os
from playerc import * 
from numpy import *

class initHandler:
    def __init__(self, project_root, project_basename, exp_cfg_data, rdf_data, fwd_coordmap, rfi, calib=False):

        ######################
        # Connect to player: #
        ######################

        print "(INIT) Connecting to Player server..."
        print "=============================================="

        # Create a client object
        self.c = playerc_client(None, rdf_data['PlayerHost'][0], int(rdf_data['PlayerPort'][0]))

        # Connect it
        if self.c.connect() != 0:
            raise playerc_error_str()

        # Create a proxy for position2d:0 (for movement commands and GPS in Stage)
        self.p = playerc_position2d(self.c,0)
        if self.p.subscribe(PLAYERC_OPEN_MODE) != 0:
            raise playerc_error_str()

        # Create a proxy for fiducial:0 (for localization in Gazebo and real life)
        #fiducial = playerc_fiducial(c,0)
        #if fiducial.subscribe(PLAYERC_OPEN_MODE) != 0:
        #    raise playerc_error_str()

        # Make it so we only get the newest data on reads
        self.c.datamode(PLAYERC_DATAMODE_PULL)
        self.c.set_replace_rule(-1, -1, PLAYER_MSGTYPE_DATA, -1, 1)

        # Retrieve the geometry
        if self.p.get_geom() != 0:
            raise playerc_error_str()
        
        print "=============================================="
        print "OK! We've successfully connected."
        
    def getSharedData(self):
        return {'PlayerClient': self.c, 
                'PlayerPos2D': self.p}
