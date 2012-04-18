# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0145090906457,-7.97493804517,-0.0163607845119,5.97177404282

InitialRegion: # Initial region number
0

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
playerstage.lab

Name: # Name of the experiment
PlayerStage

RobotFile: # Relative path of robot description file
pioneer_stage.robot


======== EXPERIMENT CONFIG 1 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.00667619043562,0.519140956528,-0.00536700273334,2.25351186904

InitialRegion: # Initial region number
0

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
cornell_asl.lab

Name: # Name of the experiment
ASL

RobotFile: # Relative path of robot description file
pioneer_real.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
pick_up,0
drop,0
radio,1
extinguish,0

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
unsynth.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
fire,1
person,1
hazardous_item,0

currentExperimentName:
ASL


======== SPECIFICATION ========

RegionMapping:

living=p5
deck=p8
porch=p4
dining=p19,p20
bedroom=p9
others=p10,p11,p12,p13,p14,p15,p16,p17,p18
kitchen=p6

Spec: # Specification in simple English
#Simple specification demonstrating liveness unrealizability
#Environment can win by alternating fire and person

Env starts with false
Robot starts with false
Robot starts in deck

Visit porch

if you are sensing fire then do not living
if you are sensing person then do not kitchen
always do not radio

always not (fire and person)

Always kitchen or porch or deck or bedroom or dining or living


