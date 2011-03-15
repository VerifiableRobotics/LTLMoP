Name: # Full name of the robot
CKBot

Sensors: # Available binary sensor propositions
None

Actions: # Available binary actuator propositions
None

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.CKBotSimController

DriveHandler:
lib.handlers.drive.CKBotDrive

ViconPort:
11111
