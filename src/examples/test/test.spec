# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.3,0.0,-0.3,0.0

InitialRegion: # Initial region number
1

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
CKBotSim.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
CKBotSim.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
pick_up,0
radio,0
drop,0
snake,1
cross,1

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
test.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
fire,0
person,1
hazardous_item,1

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

r4=p2
r3=p3
R1=p5
R2=p4
others=p6,p7,p8,p9

Spec: # Specification in simple English
If you are not sensing person then Visit R1
If you are not sensing person then Visit R2
If you are not sensing person then Visit r3
If you are not sensing person then Visit r4
Do snake if and only if you are sensing person


