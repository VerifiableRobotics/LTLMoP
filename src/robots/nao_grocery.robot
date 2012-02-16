Name: # Full name of the robot
Nao the Shopkeeper

Sensors: # Available binary sensor propositions
see_spill
see_missingitem
head_tapped

Actions: # Available binary actuator propositions
look_leftright
call_manager
sit_down
say_impossible
say_spill

MotionControlHandler: # Module with continuous controller for moving between regions
lib.handlers.motionControl.vectorController

DriveHandler: # Module for converting a desired velocity vector to realistic motor commands
lib.handlers.drive.bipedalDrive

InitializationHandler:
lib.handlers.init.naoInit

SensorHandler:
lib.handlers.sensor.naoSensor_landMark

ActuatorHandler:
lib.handlers.actuator.naoActuator_grocery

LocomotionCommandHandler:
lib.handlers.locomotionCommand.naoLocomotionCommand

### Below are settings for Nao

NaoIP:
nao.local

NaoPort:
9559

ViconName_X:
Nao:Nao <t-X>

ViconName_Y:
Nao:Nao <t-Y>

ViconName_Theta:
Nao:Nao <a-Z>
