Name: # Full name of the robot
Pioneer01 the Firefighter

Sensors: # Available binary sensor propositions
fire
person
hazardous_item

Actions: # Available binary actuator propositions
pick_up
radio
extinguish
drop

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.heatController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.differentialDrive

### Below are settings for Scorpion08

OrcaVelocityHost:
10.0.0.182

OrcaVelocityPort:
13339

OrcaPositionGroup:
239.255.255.0

OrcaPositionPort:
40976

ColorDetectionGroup:
239.255.255.3

ColorDetectionPort:
13332
