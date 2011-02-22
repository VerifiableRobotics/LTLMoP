Name: # Full name of the robot
ckdemo

Sensors: # Available binary sensor propositions
calib_wand
screwdriver
chainsaw
wire
csharp_help
advisor

Actions: # Available binary actuator propositions
viconpose
explode
grab
drop

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.CKBotSimController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.CKBotSimDrive

