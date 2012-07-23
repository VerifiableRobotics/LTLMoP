RobotName: # The name of the robot
ROS

Type: # Robot type
ROS

InitHandler: # Robot default init handler with default argument values
rosInit(calib=False)

PoseHandler: # Robot default pose handler with default argument values
viconPose(host='10.0.0.102',port=800,x_VICON_name="ros:ros <t-X>",y_VICON_name="ros:ros <t-Y>",theta_VICON_name="ros:ros <a-Z>")

SensorHandler: # Robot default sensors handler with default argument values
rosSensor()

ActuatorHandler: # Robot default actuator handler wit hdefault argument values
rosActuator()

MotionControlHandler: # Robot default motion control handler with default argument values
heatController()

DriveHandler: # Robot default drive handler with deafult argument values
rosDrive()

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
rosLocomotionCommand()
