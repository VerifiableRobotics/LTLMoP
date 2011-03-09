Name: # Full name of the robot
CKBot-predatorprey

Sensors: # Available binary sensor propositions
predator
prey
poison

Actions: # Available binary actuator propositions
fast_and_1D_motion
low_and_nonholonomic_turning
action_gait

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.CKBotSimController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.CKBotSimDrive

