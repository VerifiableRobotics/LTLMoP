Name: # Full name of the robot
Nao the Explorer stage

Sensors: # Available binary sensor propositions
TouchFront
TouchMiddle
TouchRear
BumpLeft
BumpRight

Actions: # Available binary actuator propositions
FaceLight
Greet

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665
