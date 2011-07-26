# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.5,0.0,-0.5,0.0

InitialRegion: # Initial region number
9

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
CKBotSim.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
CKBotJul11.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
carrying_person,1
taking_cover,1
T_narrow,1
T_1D_motion,1
T_low,1
T_legged,1

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
July11.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
air_raid,1
distress_signal,1

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

Rescue_Point=p8
Mountain3=p9
Mountain1=p11
Safehouse=p6
between$LeftOfTrench$and$RightOfTrench$=p4,p13
RightOfTrench=p7
Trench=p4
Trail=p5
LeftOfTrench=p12
others=p14,p15,p16,p17,p18,p19,p20
Watchtower=p3
Mountain2=p10
Cliff=p13

Spec: # Specification in simple English
Robot starts in Safehouse
Env starts with false
Always not Cliff and not Mountain1 and not Mountain2 and not Mountain3

# Rescue Mission
If you are sensing distress_signal then visit Rescue_Point
#If you are activating carrying_person or you activated carrying_person then do not distress_signal
If you are activating carrying_person or you activated carrying_person then visit Safehouse
carrying_person is set on Rescue_Point and reset on Safehouse
If you are not sensing distress_signal and you are not activating carrying_person then visit Watchtower
If you are activating carrying_person or you activated carrying_person then do not RightOfTrench

# Traits
If you are in LeftOfTrench or RightOfTrench then do not taking_cover
Do T_narrow if and only if you are in between LeftOfTrench and RightOfTrench
Do taking_cover if and only if you are sensing air_raid and you are not activating T_narrow and you did not activate T_narrow
Do T_low if and only if you are activating taking_cover
Do T_legged if and only if you are activating taking_cover


