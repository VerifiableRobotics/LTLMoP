RobotName: # Robot Name
Spider

Type: # Robot type
Spider

ActuatorHandler: # Robot default actuator handler with default argument values
SpiderActuator()

SensorHandler: # Robot default actuator handler with default argument values
SpiderSensor()

DriveHandler: # Robot default drive handler with default argument values
SpiderDrive()

InitHandler: # Robot default init handler with default argument values
SpiderInit(comPort='COM8')

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
SpiderLocomotionCommand()

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
viconPose(host='10.0.0.102',port=800,x_VICON_name="Spider:Spider <t-X>",y_VICON_name="Spider:Spider <t-Y>",theta_VICON_name="Spider:Spider <a-Z>")

