RobotName: # Robot Name
Bob

Type: # Robot type
Hexapod

ActuatorHandler: # Robot default actuator handler with default argument values
Hexapod.HexapodActuatorHandler()

SensorHandler: # Robot default actuator handler with default argument values
Hexapod.HexapodSensorHandler()

DriveHandler: # Robot default drive handler with default argument values
Hexapod.HexapodDriveHandler()

InitHandler: # Robot default init handler with default argument values
Hexapod.HexapodInitHandler(comPort="COM5")

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
Hexapod.HexapodLocomotionCommandHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.VectorControllerHandler()

PoseHandler: # Robot default pose handler with default argument values
share.Pose.ViconPoseHandler(host='10.0.0.102',port=800,x_VICON_name="Hexapod:Hexapod <t-X>",y_VICON_name="Hexapod:Hexapod <t-Y>",theta_VICON_name="Hexapod:Hexapod <a-Z>")
