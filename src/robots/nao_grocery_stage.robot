Name: # Full name of the robot
Nao the Shopkeeper (Stage)

Sensors: # Available binary sensor propositions
see_spill
see_missingitem
head_tapped

Actions: # Available binary actuator propositions
look_leftright
call_manager
sit_down
say_impossible
say_spill

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.holonomicDrive


PlayerHost:
localhost

PlayerPort:
6665
