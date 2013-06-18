RobotName: # The name of the robot
ROS

Type: # Robot type
ROS

InitHandler: # Robot default init handler with default argument values
rosInit(worldFile="ltlmop_map.world", robotPixelWidth=200, robotPhysicalWidth=.5, package="pr2_gazebo", launchFile="pr2.launch")

PoseHandler: # Robot default pose handler with default argument values
rosPose(modelName="pr2")

SensorHandler: # Robot default sensors handler with default argument values
rosSensor()

ActuatorHandler: # Robot default actuator handler wit hdefault argument values
rosActuator()

MotionControlHandler: # Robot default motion control handler with default argument values
heatController()

DriveHandler: # Robot default drive handler with deafult argument values
rosDrive(d=.3)

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
rosLocomotionCommand(velocityTopic='/base_controller/command')
