Name: # Full name of the robot
CKBot in a sandbox

Sensors: # Available binary sensor propositions
jazz
hiphop
blues
rap
country

Actions: # Available binary actuator propositions
snake
biped
plus
splits
crawl
loop
slinky

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.CKBotSimDrive

