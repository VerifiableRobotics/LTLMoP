RobotName: # Robot Name
Pioneer Real

Type: # Robot type
PioneerReal

ActuatorHandler: # Robot default actuator handler with default argument values
PioneerRealActuator()

DriveHandler: # Robot default drive handler with default argument values
differentialDrive(d=0.6)

InitHandler: # Robot default init handler with default argument values
PioneerRealInit(LocalIP='0.0.0.0',ListenerPort=6501,BroadcasterIP='10.255.255.255',BroadcasterPort=6502)

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
PioneerRealLocomotionCommand(scaleV=1.0,scaleW=1.0)

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
viconPose()

SensorHandler: # Robot default sensor handler with default argument values
PioneerRealSensor()

