# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
1.0,0.0,1.0,0.0

InitialRegion: # Initial region number
0

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
.robot


======== EXPERIMENT CONFIG 1 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
1.0,0.0,1.0,0.0

InitialRegion: # Initial region number
6

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
naoReal.lab

Name: # Name of the experiment
asl

RobotFile: # Relative path of robot description file
nao_hns.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
count,0
whistle,0
hide,1

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
testhandlers.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
see_player,0
hear_whistle,1
hear_counting,1

currentExperimentName:
asl


======== SPECIFICATION ========

RegionMapping:

Classroom1=p11
Classroom2=p10
Closet=p9
Office=p7
danger=p3
Wall=p4
Gym=p8
others=p12,p13,p14,p15,p16,p17,p18,p19,p20,p21,p22,p23,p24,p25
tree=p2
SchoolWall=p26,p27,p28,p29,p30,p31,p32,p33,p34,p35,p36,p37,p38,p39,p40,p41
Parking=p6

Spec: # Specification in simple English
robot starts with false
robot starts in Office
environment starts with false
go to Office and stay there
#do hide if and only if you are activating see_player
hide is set on hear_counting and reset on hear_whistle


