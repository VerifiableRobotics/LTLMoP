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
pioneer_ode.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
pokemon_cat.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
hmwk_given,0
hmwk_taken,0
wait_until_given,0
wait_until_taken,0

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
Feb21.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
homework,0

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

r1=p3
r2=p2
others=p4,p5,p6,p7,p8,p9,p10,p11

Spec: # Specification in simple English
visit r1
visit r2


