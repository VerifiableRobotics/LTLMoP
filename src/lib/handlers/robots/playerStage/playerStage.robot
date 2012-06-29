RobotName: # Robot Name
Player Robot 

Type: # Robot type
playerStage

ActuatorHandler: # Robot default actuator handler with default argument values
playerStageActuator()

DriveHandler: # Robot default drive handler with default argument values
holonomicDrive(multiplier=10.0,maxspeed=999.0)

InitHandler: # Robot default init handler with default argument values
playerStageInit(host='localhost',port=6665,init_region=None,Enable_Stage=True)

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
playerStageLocomotionCommand()

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
stagePose()

SensorHandler: # Robot default sensor handler with default argument values
playerStageSensor()




