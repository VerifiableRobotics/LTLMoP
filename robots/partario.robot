Name: # Full name of the robot
Partario the Conqueror

Sensors: # Available binary sensor propositions
lightOn
senseBook
senseClothes
senseTrash

Actions: # Available binary actuator propositions
pickUp
drop
destroy

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665
