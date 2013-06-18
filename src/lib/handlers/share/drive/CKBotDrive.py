#!/usr/bin/env python
"""
================================================
CKBotDrive.py - CKBot Drive Handler
================================================
"""
import math, time

class driveHandler:

    def __init__(self, proj, shared_data,gait):
        """
        Drive handler for CKBot robot
        
        gait (int): Initial gait for the robot. 0-don't move; 1-go straight; 2-go left; 3-go right(default=0)
        """
    
    
        # Get reference to locomotion command handler, since
        # we'll be passing commands right on to it
    
        self.gait = gait
        self.runtime = shared_data['Runtime']
        self.config = shared_data['Config']
        self.shared_data = shared_data
        self.pose_handler = proj.pose_handler

        try:
            self.loco = proj.loco_handler
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

    def setVelocity(self, x, y, theta=0):
        # TODO: Given x and y velocities figure out which gait should be selected. For the case of the Snake robot, for example, we choose between left, right and forward gaits here.

        # First get the current pose of the robot.
        robotangle = self.shared_data['Angle']
        #print robotangle*180.0/math.pi

        velangle = math.atan2(y,x)
        rel_heading = velangle-robotangle
        if (rel_heading > math.pi):
            rel_heading = rel_heading - 2*math.pi
        elif (rel_heading < -math.pi):
            rel_heading = rel_heading + 2*math.pi

        gait = 0
        # Check for zero (or really low) speed commands to assign no gait.
        # Only do this for the initial configuration (currently set to Hexapod)
        if (x==0 and y==0):
            gait = 0            # Do not move

        else:    

            # If there is a non-zero speed command, select among forwards, left or right turn gaits.
            if self.config=="Snake" or self.config=="Tee" or self.config=="Tripod":

                # If you were originally going straight, keep going straight until you deviate by a larger angle.
                if self.gait == 1:
                    if (rel_heading<-math.pi/6):
                        gait = 3            # Go Right
                    elif (rel_heading>math.pi/6):
                        gait = 2            # Go Left
                    else:
                        gait = 1            # Go Straight

                # Otherwise, if you were not originally going straight, only select to go straight if within a smaller angle.
                else:
                    if (rel_heading<-math.pi/12):
                        gait = 3            # Go Right
                    elif (rel_heading>math.pi/12):
                        gait = 2            # Go Left
                    else:
                        gait = 1            # Go Straight

            elif self.config == "TeeStationary":
                gait = 1

            else:
                gait = 0

        self.loco.sendCommand(gait)
        self.gait = gait

