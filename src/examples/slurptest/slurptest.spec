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
sweep,1

Customs: # List of custom propositions
m_visit_deck
m_visit_porch
m_sweep_porch
m_sweep_deck

RegionFile: # Relative path of region description file
iros10.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
fire,0
person,0
hazardous_item,0
sweep_done,1

currentExperimentName:
ASL


======== SPECIFICATION ========

RegionMapping:

living=p4
deck=p7
porch=p3
dining=p6
bedroom=p8
others=p2,p9,p10
kitchen=p5

Spec: # Specification in simple English
Search the porch and the deck.
Avoid the kitchen.
Start in the bedroom.


