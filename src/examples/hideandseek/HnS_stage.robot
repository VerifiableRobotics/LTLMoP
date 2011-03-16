Name: # Full name of the robot
Anne

Sensors: # Available binary sensor propositions
see_player
hear_whistle
hear_counting

Actions: # Available binary actuator propositions
count
whistle
hide
say_imfound
say_foundyou
say_seeker
say_hider

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.vectorController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

