Name: # Full name of the robot
CKBot-predatorprey

Sensors: # Available binary sensor propositions
predator
prey
poison

Actions: # Available binary actuator propositions
T_stationary
T_fast
T_1D_motion
T_low
T_nonholonomic_turning
T_fast_and_1D_motion
T_low_and_nonholonomic_turning

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.CKBotSimController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.CKBotSimDrive

