RobotName: # Robot Name
Basic Simulated Robot

Type: # Robot type
basicSim

ActuatorHandler: # Robot default actuator handler with default argument values

DriveHandler: # Robot default drive handler with default argument values
share.Drive.HolonomicDriveHandler(multiplier=50.0,maxspeed=999.0)

InitHandler: # Robot default init handler with default argument values
basicSim.BasicSimInitHandler()

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
basicSim.BasicSimLocomotionCommandHandler(speed=1.0)

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.VectorControllerHandler()

PoseHandler: # Robot default pose handler with default argument values
basicSim.BasicSimPoseHandler()

SensorHandler: # Robot default sensor handler with default argument values

CalibrationMatrix:
array([[ 1, 0, 0],
       [ 0, 1, 0],
       [ 0, 0, 1]])


