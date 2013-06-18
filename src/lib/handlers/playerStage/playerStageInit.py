#!/usr/bin/env python
"""
======================================================
playerclient.py - Player Client Initialization Handler
======================================================

Creates a player client, connects to a specified player server, and instantiates a Position2D proxy object

Returns the player client object and Position2D proxy object
"""

import sys, time
from playerc import * 
import _stage

class initHandler:
    def __init__(self, proj, host, port,init_region,Enable_Stage):
        """
        Creates a player client, connects to a specified player server, and instantiates a Position2D proxy object
        Returns the player client object and Position2D proxy object

        host (string): The ip address of player (default="localhost")
        port (int): The port of player (default=6665)
        init_region (region): The name of initial region
        Enable_Stage (bool): True for enabling stage simulator (default=True)
        """

        print "(INIT) Connecting to Player server..."
        print "=============================================="

        # Create a client object
        try:
            self.c = playerc_client(None, host, port)
        except KeyError:
            print "(INIT) ERROR: 'PlayerHost' and 'PlayerPort' must be specified for init handler"
            sys.exit(-1)
            
        # initialize state
        if Enable_Stage:
            _stage.initHandler(proj,init_region)

        # Connect it
        for timeout in range(0,5):
            if self.c.connect() != 0:
                print playerc_error_str()
                if timeout < 2:
                    print "(INIT) Retrying..."
                    time.sleep(3)
                else:
                    print "(INIT) Connection could not be established.  Aborting."
                    sys.exit(-1)
            else:
                break;

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
        """ Return the Player Client object to be used by handlers further down the line"""

        return {'PlayerClient': self.c,
                'PlayerPos2D': self.p}
