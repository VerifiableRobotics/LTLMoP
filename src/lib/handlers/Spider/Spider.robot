RobotName: # Robot Name
Spider

Type: # Robot type
Spider

ActuatorHandler: # Robot default actuator handler with default argument values
Spider.SpiderActuatorHandler()

SensorHandler: # Robot default actuator handler with default argument values
Spider.SpiderSensorHandler()

DriveHandler: # Robot default drive handler with default argument values
Spider.SpiderDriveHandler()

InitHandler: # Robot default init handler with default argument values
Spider.SpiderInitHandler(comPort='COM8')

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
Spider.SpiderLocomotionCommandHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.VectorControllerHandler()

PoseHandler: # Robot default pose handler with default argument values
share.Pose.ViconPoseHandler(host='10.0.0.102',port=800,x_VICON_name="Spider:Spider <t-X>",y_VICON_name="Spider:Spider <t-Y>",theta_VICON_name="Spider:Spider <a-Z>")

