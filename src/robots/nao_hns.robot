Name: # Full name of the robot
Nao

Sensors: # Available binary sensor propositions
see_player
hear_whistle
hear_counting

Actions: # Available binary actuator propositions
count
whistle
hide
say_foundyou
say_imfound
say_hider
say_seeker

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
