RobotName: # Robot Name
NXT

Type: # Robot type
NXT

ActuatorHandler: # Robot default actuator handler with default argument values
NXTActuator()

DriveHandler: # Robot default drive handler with default argument values
NXTDrive()

InitHandler: # Robot default init handler with default argument values
NXTInit(brick='NXT', brickMAC='00:16:53:14:1B:33')

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
NXTLocomotionCommand(leftDriveMotor='PORT_B',rightDriveMotor='PORT_C',steeringMotor='none',leftForward=True,rightForward=True)

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
viconPose(host='10.0.0.102',port=800,x_VICON_name="NXT:NXT <t-X>",y_VICON_name="NXT:NXT <t-Y>",theta_VICON_name="NXT:NXT <a-Z>")

SensorHandler: # Robot default sensor handler with default argument values
NXTSensor()