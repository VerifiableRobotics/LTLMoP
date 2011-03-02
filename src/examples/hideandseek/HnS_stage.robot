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

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

