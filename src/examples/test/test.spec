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
T_stationary,1
T_fast,1
T_1D_motion,1
T_low,1
T_nonholonomic_turning,1
T_fast_and_1D_motion,1
T_low_and_nonholonomic_turning,1

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
test.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
predator,1
prey,1
poison,1

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

R1=p2
R2=p1
others=

Spec: # Specification in simple English
Visit R1
Visit R2
If you are sensing predator then do T_fast
If you are in R2 then do T_1D_motion


