RobotName: # Robot Name
Basic Simulated Robot

Type: # Robot type
basicSim

ActuatorHandler: # Robot default actuator handler with default arguement values
basicSimActuator()

DriveHandler: # Robot default drive handler with default arguement values
holonomicDrive(multiplier=1.0,maxspeed=999.0)

InitHandler: # Robot default init handler with default arguement values
basicSimInit(initial_region=None)

LocomotionCommandHandler: # Robot locomotion command actuator handler with default arguement values
basicSimLocomotionCommand(speed=1.0)

MotionControlHandler: # Robot default motion control handler with default arguement values
vectorController()

PoseHandler: # Robot default pose handler with default arguement values
baiscSimPose()

SensorHandler: # Robot default sensor handler with default arguement values
basicSimSensor()




