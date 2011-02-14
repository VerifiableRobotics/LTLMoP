# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
1.0,0.0,1.0,0.0

InitialRegion: # Initial region number
5

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
playerstage.lab

Name: # Name of the experiment
Default

RobotFile: # Relative path of robot description file
HnS_stage.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
Clean,1
Fold,1

Customs: # List of custom propositions
been_found
counting
seeker

RegionFile: # Relative path of region description file
hideandseek.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
found,1
whistle,1

currentExperimentName:
Default


======== SPECIFICATION ========

RegionMapping:

Classroom1=p14
Classroom2=p13
Closet=p12
Office=p9,p59,p60
Danger=p61,p62
Wall=p4
Gym=p10
others=p15,p16,p17,p18,p19,p20,p21,p22,p23,p24,p25,p26,p27,p28,p29,p30,p31,p32,p33,p34,p35,p36,p37
Tree=p5
SchoolWall=p9,p43,p44,p45,p46,p47,p48,p49,p50,p51,p52,p53,p54,p55,p56,p57,p58
Parking=p7
between$Tree$and$Wall$=p38,p39,p40,p41,p42

Spec: # Specification in simple English
# Initialization
Environment starts with false
Robot starts in Parking

# Restrictions
Always not Danger or SchoolWall or Wall or Tree

# Hiding behavior
If you are not activating seeker then go to ((between Tree and Wall) or Closet or Office)
if you are not activating seeker and you were in ((between Tree and Wall) or Closet or Office) then stay there
been_found is set on found and not seeker and reset on counting
If been_found then visit Parking
counting is set on Parking and been_found and reset on whistle

# Seeking behavior
seeker is set on whistle and reset on found
If seeker then visit Parking and Office and Gym and Classroom1 and Classroom2 and Closet and (between Tree and Wall)

