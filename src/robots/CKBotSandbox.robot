Name: # Full name of the robot
CKBot-sandbox

Sensors: # Available binary sensor propositions
lowclearance
fire
shovel
music

Actions: # Available binary actuator propositions
grab
drop
cross
slinky
splits
action_gait
low_and_holonomic
handles_steps
handles_fire

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.CKBotSimController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.CKBotSimDrive

