Name: # Full name of the robot
frog

Sensors: # Available binary sensor propositions
Fly
Bird
Snake
Human

Actions: # Available binary actuator propositions
PickUp
Drop

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

