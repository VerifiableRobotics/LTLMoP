Name: # Full name of the robot
Anne

Sensors: # Available binary sensor propositions
been_found
whistle
found_target

Actions: # Available binary actuator propositions
hiding
seeking
counting

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.vectorController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

