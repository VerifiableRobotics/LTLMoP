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


Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file



======== EXPERIMENT CONFIG 1 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.65,0.0,-0.65,0.0

InitialRegion: # Initial region number
12

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
CKBotSim.lab

Name: # Name of the experiment
CKBotSim

RobotFile: # Relative path of robot description file
CKBotPredatorPrey.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
Tfast_and_1D_motion,1
Tlow_and_nonholonomic_turning,1
Tstationary,1

Customs: # List of custom propositions
Prancing

RegionFile: # Relative path of region description file
IROS11.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
predator,1
prey,1
poison,1

currentExperimentName:
CKBotSim


======== SPECIFICATION ========

RegionMapping:

Mountain2=p9
Mountain=p10
Bridge=p15
Field=p13
Tunnel=p7
Island=p12
Water3=p3
Water2=p4,p5
between$Island$and$Dock$=p5,p15
Water=p6
Dock=p14
others=p2,p16,p17
Springs=p8
Meadows=p11

Spec: # Specification in simple English
Env starts with false
Robot starts in Island
Always not Water and not Water2 and not Water3
If you were in Tunnel then do not sense predator or prey
If you were in between Island and Dock then do not sense predator or prey
If you are not sensing predator and you are not sensing prey and you are not sensing poison then visit Meadows
If you are not sensing predator and you are not sensing prey and you are not sensing poison then visit Dock
If you are not sensing predator and you are not sensing prey and you are sensing poison then visit Springs
Do Tlow_and_nonholonomic_turning if and only if you are in Tunnel
Tfast_and_1D_motion is set on between Island and Dock and reset on Dock
If you are sensing predator then stay there
If you are sensing prey then do Tstationary
Do Prancing if and only if you are in Meadows


