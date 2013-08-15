RobotName: # Robot Name
Pioneer with Homework Sensor

Type: # Robot type
CSharpRobot

InitHandler: # Robot default init handler with default argument values
CSharpRobotInit(robotType = 2,IPAddress = '10.0.0.91',commPort=7400,buffer=2048576)

DriveHandler:
differentialDrive()

SensorHandler:
CSharpRobotSensor()

ActuatorHandler:
CSharpRobotActuator()

MotionControlHandler: # Module with continuous controller for moving between regions
vectorController()

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
CSharpRobotLocomotionCommand()

PoseHandler: # Robot default pose handler with default argument values
viconPose(host='10.0.0.102',port=800,x_VICON_name="yellowsegway01:yellowsegway01 <t-X>",y_VICON_name="yellowsegway01:yellowsegway01 <t-Y>",theta_VICON_name="yellowsegway01:yellowsegway01 <a-Z>")