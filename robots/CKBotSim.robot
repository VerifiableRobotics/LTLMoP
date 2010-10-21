Name: # Full name of the robot
CKBot-Simulated

Sensors: # Available binary sensor propositions
fire
person
hazardous_item

Actions: # Available binary actuator propositions
pick_up
radio
drop
snake
cross

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler:
handlers.drive.CKBotSimDrive
