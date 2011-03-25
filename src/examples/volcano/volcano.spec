# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.00382540920792,-0.781606310715,-0.00371294076761,0.881904731677

InitialRegion: # Initial region number
8

InitialTruths: # List of initially true propositions

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

RegionFile: # Relative path of region description file
volcano.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
Landslide,1
Burn,1

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

Plains=p4
Lava3=p6
Lava2=p7
Lava1=p8
others=
between$Lava1$and$Lava2$=p1,p2
Lava4=p5
Trail=p2
Volcano=p1
Springs=p3

Spec: # Specification in simple English
Env starts with false
Robot starts in Plains
Always do T_hardware
If you are not sensing Landslide and you are not sensing Burn then visit Volcano
If you are not sensing Landslide and you are not sensing Burn then visit Plains
If you are not sensing Landslide and you are sensing Burn then visit Springs
Always not Lava1 and not Lava2 and not Lava3 and not Lava4
If you were in between Lava1 and Lava2 then do not sense Landslide
T_low is set on between Lava1 and Lava2 and reset on Plains or Springs
T_narrow is set on between Lava1 and Lava2 and reset on Plains or Springs
If you are sensing Landslide then stay there
If you are in Springs and you are sensing Burn then do T_stationary


