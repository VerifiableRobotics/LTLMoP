RobotName: # Robot Name
Basic Simulated Robot

Type: # Robot type
basicSim

ActuatorHandler: # Robot default actuator handler with default argument values
basicSimActuator()

DriveHandler: # Robot default drive handler with default argument values
holonomicDrive(multiplier=50.0,maxspeed=999.0)

InitHandler: # Robot default init handler with default argument values
basicSimInit(init_region=None)

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
basicSimLocomotionCommand(speed=1.0)

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
basicSimPose()

SensorHandler: # Robot default sensor handler with default argument values
basicSimSensor()

CalibrationMatrix:
array([[ 1, 0, 0],
       [ 0, 1, 0],
       [ 0, 0, 1]])


