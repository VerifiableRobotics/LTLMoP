Name: # Full name of the robot
Charlie

Sensors: # Available binary sensor propositions
sheep
wolf

Actions: # Available binary actuator propositions
start_chase
set_free
bark

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

