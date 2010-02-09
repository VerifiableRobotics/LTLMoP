Name: # Full name of the robot
Pioneer upon Avon

Sensors: # Available binary sensor propositions
fire
person
hazardous_item

Actions: # Available binary actuator propositions
pick_up
radio
extinguish
drop

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

