#!/usr/bin/env python
"""
==================================================================
Johnny5LocomotionCommandHandler.py - Johnny5 Locomotion Command Handler
==================================================================

Send commands to Johnny 5.
"""
import sys
import math
import logging
import globalConfig

import lib.handlers.handlerTemplates as handlerTemplates

class Johnny5LocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    def __init__(self, executor, ang_vel_gain, fwd_vel_gain, min_ang_vel, max_ang_vel, min_fwd_vel, max_fwd_vel, shared_data):
        """
        Locomotion Command handler for Johnny 5 robot.

        min_ang_vel (double): Lower bound of angular velocity (default=0.1)
        max_ang_vel (double): Upper bound of angular velocity (default=0.5)
        min_fwd_vel (double): Lower bound of forward velocity (default=0.1)
        max_fwd_vel (double): Upper bound of forward velocity (default=0.3)
        ang_vel_gain (int): The value to scale the angular velocity command, larger means faster. (default=700)
        fwd_vel_gain (int): The value to scale the forward velocity command, larger means faster. (default=700)
        """

        # find johnny5 serial port
        try:
            self.johnny5Serial = shared_data["Johnny5Serial"]
        except:
            logging.exception("No connection to Johnny 5")
            sys.exit(-1)

        # set upper bound and lower bound for velocity
        self.min_ang_vel = min_ang_vel
        self.max_ang_vel = max_ang_vel
        self.min_fwd_vel = min_fwd_vel
        self.max_fwd_vel = max_fwd_vel
        self.ang_vel_gain = ang_vel_gain
        self.fwd_vel_gain = fwd_vel_gain

        # load config info
        config = shared_data['DefaultConfig']

        # load servo #14 and #15 neutral value
        self.ang_servo_neutral = config[15][0]
        self.fwd_servo_neutral = config[14][0]


    def sendCommand(self, cmd):
        """
        Send movement command to Johnny 5
        """

        # Angular velocity value, set lower and upper bound
        if cmd[1]!=0 and math.fabs(cmd[1])<self.min_ang_vel:
            cmd[1] = math.copysign(self.min_ang_vel, cmd[1])
        if math.fabs(cmd[1])>self.max_ang_vel:
            cmd[1] = math.copysign(self.max_ang_vel, cmd[1])

        # Forward velocity value, set lower and upper bound
        if cmd[0]!=0 and math.fabs(cmd[0])<self.min_fwd_vel:
            cmd[0] = math.copysign(self.min_fwd_vel, cmd[0])
        if math.fabs(cmd[0])>self.max_fwd_vel:
            cmd[0] = math.copysign(self.max_fwd_vel, cmd[0])


        # print debugging message
        logging.debug("Angular Velocity: {}".format(cmd[1]))
        logging.debug("Forward Velocity: {}".format(cmd[0]))

        # Send command to johnny5
        self.johnny5Serial.write('#15 P%d\r' % (self.ang_servo_neutral-self.ang_vel_gain*cmd[1]))
        self.johnny5Serial.write('#14 P%d\r' % (self.fwd_servo_neutral-self.fwd_vel_gain*cmd[0]))

