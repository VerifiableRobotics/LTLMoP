Name: # Full name of the robot
Some Triangle

Sensors: # Available binary sensor propositions
been_found
found_target
whistle

Actions: # Available binary actuator propositions
counting
seeking
hiding

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.vectorController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.bipedalDrive

### Below are settings for Nao

NaoIP:
nao.local

NaoPort:
9559

ViconPort:
9560
