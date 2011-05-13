# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.8,0.0,-0.8,0.0

InitialRegion: # Initial region number
5

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
CKBotSim.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
CKBotMay11.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
T_stationary,1
T_fast,1
T_1D_motion,1
T_low,1
T_nonholonomic_turning,1
T_narrow,1
T_legged,1
T_large,1

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

Wall1=p2
Enemy_Base=p10
Mountains=p7
Tunnel=p4
others=
Trench=p5
Wall2=p1
Jungle=p8
Bunker=p11
WAR_ZONE=p3
Gate=p9
Supply_Depot=p6

Spec: # Specification in simple English
Env starts with false
Robot starts in Supply_Depot
Always not Wall1 and not Wall2 and not Mountains
If you are not sensing supply and you are not sensing infiltrate then visit Bunker
If you are sensing supply and you are not sensing infiltrate then visit Supply_Depot
If you are sensing infiltrate then visit Enemy_Base
Do not WAR_ZONE unless you are sensing invincible
If you are sensing supply and you are in Supply_Depot then visit Bunker
If you are sensing infiltrate and you are in Enemy_Base then visit Bunker
Do T_fast and do T_1D_motion if and only if you are in Trench
Do T_legged and T_large if and only if you are in WAR_ZONE or Enemy_Base or Supply_Depot
Do T_low and do T_legged if and only if you are in Jungle


