RobotName: # Robot Name
OptimusSecundus

Type: # Robot type
iRobotCreate

InitHandler: # Robot default init handler with default argument values
iRobotCreate.IRobotCreateInitHandler(listenIP = '0.0.0.0',broadCastIP = '192.168.1.120',createPort=8865,beaglePort=8866,artagPort=8844,sonarPort=8833,buffer=1024)

DriveHandler:
share.Drive.DifferentialDriveHandler()

SensorHandler:
iRobotCreate.IRobotCreateSensorHandler()

ActuatorHandler:
iRobotCreate.IRobotCreateActuatorHandler()

MotionControlHandler: # Module with continuous controller for moving between regions
share.MotionControl.VectorControllerHandler()

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
iRobotCreate.IRobotCreateLocomotionCommandHandler()

PoseHandler: # Robot default pose handler with default argument values
share.Pose.ViconPoseHandler(host='10.0.0.102',port=800,x_VICON_name="roomba_no_arm:roomba_no_arm <t-X>",y_VICON_name="roomba_no_arm:roomba_no_arm <t-Y>",theta_VICON_name="roomba_no_arm:roomba_no_arm <a-Z>")
