# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
1.0,0.0,1.0,0.0

InitialRegion: # Initial region number
-1

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
.robot


======== EXPERIMENT CONFIG 1 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.3,0.0,-0.3,0.0

InitialRegion: # Initial region number
1

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
pioneer_ode.lab

Name: # Name of the experiment
New Experiment1

RobotFile: # Relative path of robot description file
pioneer_ode.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
radio,1
pick_up,0
drop,0

Customs: # List of custom propositions
imacustom

RegionFile: # Relative path of region description file
newSensorTest.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
hazardous_item,0
person,1

currentExperimentName:
New Experiment1


======== SPECIFICATION ========

RegionMapping:

r3=p1
r1=p3
r2=p2
others=

Spec: # Specification in simple English
visit r1
visit r2
visit r3
do radio if and only if you are sensing person


