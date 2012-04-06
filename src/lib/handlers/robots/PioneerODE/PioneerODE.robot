RobotName: # Robot Name
Pioneer ODE

Type: # Robot type
PioneerODE

ActuatorHandler: # Robot default actuator handler with default argument values
PioneerODEActuator()

DriveHandler: # Robot default drive handler with default argument values
differentialDrive(d=0.65)

InitHandler: # Robot default init handler with default argument values
PioneerODEInit()

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
PioneerODELocomotionCommand()

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
pioneerSimPose()

SensorHandler: # Robot default sensor handler with default argument values
PioneerODESensor()

CalibrationMatrix:
array([[  3.3333,       0, 0],
       [       0, -3.3333, 0],
       [       0,       0, 1]])



