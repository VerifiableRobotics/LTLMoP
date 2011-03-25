Name: # Full name of the robot
CKBot

Sensors: # Available binary sensor propositions
Landslide
Burn

Actions: # Available binary actuator propositions
T_narrow
T_low
T_hardware
T_stationary

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.CKBotSimController

DriveHandler:
lib.handlers.drive.CKBotDrive

ViconPort:
11111
