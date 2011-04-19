# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.00656339232415,3.54995774955,-0.00722948261407,2.04105576833

InitialRegion: # Initial region number
6

InitialTruths: # List of initially true propositions
T_hardware

Lab: # Lab configuration file
CKBot.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
CKBot.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
T_narrow,1
T_low,1
T_hardware,1
T_stationary,1

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
volcano.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
Landslide,1
Burn,1

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

Lava3=p6
Lava2=p7
Lava1=p8
between$Lava1$and$Lava2$=p1,p2
Plains=p5
Springs=p3
Plateau=p4
others=
between$Plains$and$Springs$=p2,p8

Spec: # Specification in simple English
Env starts with false
Robot starts in Plains
Always do T_hardware
If you are not sensing Landslide and you are not sensing Burn then visit Plateau
If you are not sensing Landslide and you are not sensing Burn then visit Plains
If you are not sensing Landslide and you are sensing Burn then visit Springs
If you were in between Plains and Springs then do not Springs
Do T_stationary if and only if you are in Springs and you are sensing Burn
Always not Lava1 and not Lava2 and not Lava3
If you were in between Lava1 and Lava2 then do not sense Landslide
T_low is set on between Lava1 and Lava2 and reset on Plains or Plateau or Springs
T_narrow is set on between Lava1 and Lava2 and reset on Plains or Plateau or Springs
If you are sensing Landslide then stay there


