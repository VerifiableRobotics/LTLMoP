RobotName: # Robot Name
Charlie

Type: # Robot type
PioneerODE

ActuatorHandler: # Robot default actuator handler with default argument values

DriveHandler: # Robot default drive handler with default argument values
share.Drive.DifferentialDriveHandler(d=0.65)

InitHandler: # Robot default init handler with default argument values
PioneerODE.PioneerODEInitHandler()

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
PioneerODE.PioneerODELocomotionCommandHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.VectorControllerHandler()

PoseHandler: # Robot default pose handler with default argument values
PioneerODE.PioneerSimPoseHandler()

SensorHandler: # Robot default sensor handler with default argument values

CalibrationMatrix:
array([[  3.3333,       0, 0],
       [       0, -3.3333, 0],
       [       0,       0, 1]])



