ActuatorHandler: # Actuator handler file in robots/Type folder
pragbotActuator()

DriveHandler: # Input value for robot drive handler, refer to file inside the handlers/drive folder
holonomicDrive(multiplier=1.0,maxspeed=999.0)

InitHandler: # Input value for robot init handler, refer to the init file inside the handlers/robots/Type folder
pragbotInit()

LocomotionCommandHandler: # Input value for robot locomotion command handler, refer to file inside the handlers/robots/Type folder
nullLocomotionCommand()

MotionControlHandler: # Input value for robot motion control handler, refer to file inside the handlers/motionControl folder
pragbotMotion()

PoseHandler: # Input value for robot pose handler, refer to file inside the handlers/pose folder
pragbotPose()

RobotName: # Robot Name
Junior

SensorHandler: # Sensor handler file in robots/Type folder
pragbotSensor()

Type: # Robot type
pragbot