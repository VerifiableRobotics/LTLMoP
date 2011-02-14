Name: # Full name of the robot
HideNSeek

Sensors: # Available binary sensor propositions
found
whistle

Actions: # Available binary actuator propositions


MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

