# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.004659,-0.792893,-0.003667,2.247108

InitialRegion: # Initial region number
4

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
iRobot_Create.lab

Name: # Name of the experiment
iRobotCreateTest

RobotFile: # Relative path of robot description file
iRobotCreate.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
sing,0
flash_LED,0
pickup,1
drop,1

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
iRobotCreate.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
bump_right,1
bump_left,1
wheelDrop_right,0
wheelDrop_left,0
wheelDrop_caster,0
wall,0
virtual_wall,0
cliff_left,0
cliff_right,0
cliffFront_left,0
cliffFront_right,0
button_play,0
button_advance,0

currentExperimentName:
iRobotCreateTest


======== SPECIFICATION ========

RegionMapping:

r4=p1
others=
r1=p4
r2=p3
r3=p2

Spec: # Specification in simple English
#Env starts with false
Robot starts in r1

If you are activating bump_right or bump_left then stay
#If you are activating bump_left then stay
If you are not activating bump_right or bump_left then visit r1
If you are not activating bump_right or bump_left then visit r2
If you are not activating bump_right or bump_left then visit r3
If you are not activating bump_right or bump_left then visit r4

Do pickup if and only if you are activating bump_right
Do drop if and only if you are activating bump_left


