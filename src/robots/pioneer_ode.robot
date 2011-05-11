Name: # Full name of the robot
Pioneer-Simulated

Sensors: # Available binary sensor propositions
hazardous_item
person

Actions: # Available binary actuator propositions
radio
pick_up
drop

MotionControlHandler: # Module with continuous controller for moving between regions
handlers.motionControl.heatController

DriveHandler:
handlers.drive.differentialDrive
