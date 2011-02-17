Name: # Full name of the robot
Golf Ball Collector

Sensors: # Available binary sensor propositions
ball
divot

Actions: # Available binary actuator propositions
pick_up
drop
fix

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

