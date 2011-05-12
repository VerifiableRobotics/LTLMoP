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



======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
T_stationary,1
T_fast,1
T_1D_motion,1
T_low,1
T_nonholonomic_turning,1
T_fast_and_1D_motion,0
T_low_and_nonholonomic_turning,0

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
may11.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
supply,1
invincible,1
infiltrate,1

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

Mountains=p8
Enemy_Base=p10
Tunnel=p5
WAR_ZONE=p4
Wall1=p3
Wall2=p2
Jungle=p9
others=p1
Bunker=p11
Trench=p6
Supply_Depot=p7

Spec: # Specification in simple English
Env starts with false
Robot starts in Supply_Depot
Always not Wall1 and not Wall2 and not Mountains
Visit Bunker if not supply
#Do T_fast and do T_1D_motion if and only if you are in Trench
#Do T_low and do T_narrow and do T_nonholonomic_turning if and only if you are in Tunnel
Do T_low and do T_nonholonomic_turning if and only if you are in Jungle


