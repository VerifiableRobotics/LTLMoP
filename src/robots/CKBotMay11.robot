Name: # Full name of the robot
CKBot-may11

Sensors: # Available binary sensor propositions
supply
invincible
infiltrate

Actions: # Available binary actuator propositions
T_stationary
T_fast
T_1D_motion
T_low
T_nonholonomic_turning
T_narrow
T_legged
T_large

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.CKBotSimController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.CKBotSimDrive

