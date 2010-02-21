#!/usr/bin/env python
"""
====================================================================
handlers/init/playerclient.py - Player Client Initialization Handler
====================================================================

Creates a player client, connects to a specified player server, and instantiates a Position2D proxy object

Returns the player client object and Position2D proxy object
"""

import sys, time
from playerc import * 

class initHandler:
    def __init__(self, proj, calib=False):
        """
        The hostname and port of the player server are loaded from the robot configuration file.

        ``calib`` is ignored.
        """

        print "(INIT) Connecting to Player server..."
        print "=============================================="

        # Create a client object
        try:
            self.c = playerc_client(None, proj.robot_data['PlayerHost'][0], int(proj.robot_data['PlayerPort'][0]))
        except KeyError:
            print "(INIT) ERROR: 'PlayerHost' and 'PlayerPort' must be specified in robot file"
            sys.exit(-1)

        # Connect it
        for timeout in range(0,3):
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
