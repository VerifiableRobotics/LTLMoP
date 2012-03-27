RobotName: # Robot Name
Player Robot 

Type: # Robot type
playerStage

ActuatorHandler: # Robot default actuator handler with default arguement values
playerStageActuator()

DriveHandler: # Robot default drive handler with default arguement values
holonomicDrive()

InitHandler: # Robot default init handler with default arguement values
playerStageInit(host='localhost',port=6665,Enable_Stage=True):

LocomotionCommandHandler: # Robot locomotion command actuator handler with default arguement values
playerStageLocomotionCommand()

MotionControlHandler: # Robot default motion control handler with default arguement values
vectorController()

PoseHandler: # Robot default pose handler with default arguement values
stagePose()

SensorHandler: # Robot default sensor handler with default arguement values
playerStageSensor()




