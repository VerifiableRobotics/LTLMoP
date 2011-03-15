Name: # Full name of the robot
CKBot-predatorprey

Sensors: # Available binary sensor propositions
predator
prey
poison

Actions: # Available binary actuator propositions
Tfast_and_1D_motion
Tlow_and_nonholonomic_turning
Tstationary

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.CKBotSimController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.CKBotSimDrive

