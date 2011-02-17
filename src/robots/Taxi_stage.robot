Name: # Full name of the robot
Taxi

Sensors: # Available binary sensor propositions
RedLight
GreenLight
Person
Jaywalker

Actions: # Available binary actuator propositions
PickUp
DropOff

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
handlers.drive.holonomicDrive

PlayerHost:
localhost

PlayerPort:
6665

