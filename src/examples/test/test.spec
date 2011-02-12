# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 1 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0154202897192,-7.84324650027,-0.0145000006471,5.46750031625

InitialRegion: # Initial region number
1

InitialTruths: # List of initially true propositions
Bird

Lab: # Lab configuration file
cornell_asl.lab

Name: # Name of the experiment
Pioneer @ ASL

RobotFile: # Relative path of robot description file
pioneer_real.robot


======== EXPERIMENT CONFIG 2 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0159613530417,-6.23252142771,-0.0140000006982,2.42610786642

InitialRegion: # Initial region number
1

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
playerstage.lab

Name: # Name of the experiment
Stage (Point Robot)

RobotFile: # Relative path of robot description file
partario.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
PickUp,1
Drop,1
Jump,0
Ponder,0

Customs: # List of custom propositions
CarryingFly

RegionFile: # Relative path of region description file
test.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
Fly,1
Bird,0
Snake,0
Human,0

currentExperimentName:
Stage (Point Robot)


======== SPECIFICATION ========

Spec: # Specification in simple English
Env starts with false
Robot starts with false
Do PickUp if and only if you are sensing Fly and you are not activating CarryingFly
Do Drop if and only if you are activating CarryingFly and you were in mystery
If you are activating PickUp or you activated PickUp then stay there
If you are activating Drop or you activated Drop then stay there
If you activated PickUp then do CarryingFly
If you activated Drop then do not CarryingFly
If you activated CarryingFly and you did not activate Drop then do CarryingFly
If you did not activate CarryingFly and you did not activate PickUp then do not CarryingFly
If you were in mystery then do not Fly
If you did not activate CarryingFly then always not mystery
If you are not activating CarryingFly then visit hash_brown
If you are not activating CarryingFly then visit bacon
If you are not activating CarryingFly then visit eggy
If you are not activating CarryingFly then visit mushrooms
If you are not activating CarryingFly then visit toast
If you are not activating CarryingFly then visit beans
If you are not activating CarryingFly then visit tomato
If you are activating CarryingFly then visit mystery

