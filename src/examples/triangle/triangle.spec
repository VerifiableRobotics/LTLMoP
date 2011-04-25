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
HnS_stage.robot


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
counting,0
seeking,1
hiding,0

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
hideandseek.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
been_found,1
found_target,0
whistle,0

currentExperimentName:
ASL


======== SPECIFICATION ========

RegionMapping:

r1=p2
r2=p1
others=

Spec: # Specification in simple English
Robot starts with seeking

If you activated seeking then do not seeking
If you were not activating seeking and you are sensing been_found then do seeking
If you were not activating seeking and you are sensing been_found then do not seeking

