Name: # Full name of the robot
Peter Rabbit

Sensors: # Available binary sensor propositions
bear
fruit

Actions: # Available binary actuator propositions
pick_up
drop
play_dead
sing

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

