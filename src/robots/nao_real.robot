Name: # Full name of the robot
Nao the Explorer

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
lib.handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.bipedalDrive

### Below are settings for Nao

NaoIP:
nao.local

NaoPort:
9559

ViconPort:
9560
