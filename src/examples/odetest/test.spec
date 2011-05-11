# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.3,0.0,-0.3,0.0

InitialRegion: # Initial region number
1

InitialTruths: # List of initially true propositions
radio
fire
person

Lab: # Lab configuration file
pioneer_ode.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
pioneer_ode.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
pick_up,0
radio,1
drop,0
snake,0
cross,0

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
test.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
fire,1
person,1
hazardous_item,0

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

r4=p2
others=p6,p7,p8,p9
R1=p5
R2=p4
r3=p3

Spec: # Specification in simple English
If you are not sensing fire then Visit R2
If you are sensing fire then Visit R1
Do radio if and only if you are sensing person


