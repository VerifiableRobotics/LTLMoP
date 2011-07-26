Name: # Full name of the robot
CKBot-Jul11

Sensors: # Available binary sensor propositions
air_raid
distress_signal

Actions: # Available binary actuator propositions
carrying_person
taking_cover
T_narrow
T_1D_motion
T_low
T_legged

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.CKBotSimController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.CKBotSimDrive

