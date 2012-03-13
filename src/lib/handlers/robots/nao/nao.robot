DriveHandler: # Robot default drive handler with default arguement values
bipedalDrive()

InitHandler: # Robot default init handler with default arguement values
naoInit(ip='nao.local',port=9559)

LocomotionCommandHandler: # Robot default locomotion command handler with default arguement values
naoLocomotionCommand()

MotionControlHandler: # Robot default motion control handler with default arguement values
vectorController()

PoseHandler: # Robot default pose handler with default arguement values
naoPose(viconPort=9560)

RobotName: # Robot Name
MAE

Type: # Robot type
nao

SensorHandler: # Robot default sensor handler with default arguement values
naoSensor()

ActuatorHandler: # Robot default actuator handler with default arguement values
naoActuator()
