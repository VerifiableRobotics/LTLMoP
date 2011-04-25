# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.012811806809,-8.07774146808,-0.0185568627189,5.91272155958

InitialRegion: # Initial region number
10

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
playerstage.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
Ben.robot


======== EXPERIMENT CONFIG 1 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0100832261975,-3.10740369646,-0.0090486321417,2.46124893359

InitialRegion: # Initial region number
10

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
naoReal.lab

Name: # Name of the experiment
ASL

RobotFile: # Relative path of robot description file
nao_hns.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
pick_up,0
drop,1
radio,0
extinguish,0

Customs: # List of custom propositions
seeking
hiding
counting

RegionFile: # Relative path of region description file
hideandseek.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
found_target,1
been_found,1
whistle,1

currentExperimentName:
ASL


======== SPECIFICATION ========

RegionMapping:

Classroom1=p11
Classroom2=p10
Office=p6
Closet=p9
Danger=p8
Wall=p2
Gym=p7
others=p12,p13,p14,p15,p16,p17,p18,p19,p20,p21,p22,p23,p24,p25
Tree=p3
SchoolWall=p26,p27,p28,p29,p30,p31,p32,p33,p34,p35,p36
Parking=p5

Spec: # Specification in simple English
Robot starts with counting and not seeking and not hiding
Always not seeking or not hiding
Always not hiding or not counting
Always not counting or not seeking
Always not whistle or not found_target
Always not found_target or not been_found
Always not been_found or not whistle
seeking is set on whistle and reset on found_target
hiding is set on found_target and reset on been_found
counting is set on been_found and reset on whistle


