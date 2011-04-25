# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0145090906457,-7.97493804517,-0.0163607845119,5.97177404282

InitialRegion: # Initial region number
2

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
2

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
playerstage.lab

Name: # Name of the experiment
ASL

RobotFile: # Relative path of robot description file
pioneer_stage.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
pick_up,1
drop,0
radio,1
extinguish,0

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
iros10.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
fire,0
person,1
hazardous_item,1

currentExperimentName:
PlayerStage


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
# Initial conditions
Env starts with false
Robot starts with false
Robot starts in porch
# Assumptions about the environment
If you were in porch then do not hazardous_item
# Define robot safety including how to pick up
Do pick_up if and only if you are sensing hazardous_item and you are not activating carrying_item
If you did not activate carrying_item then always not porch
# Define when and how to radio 
Do radio if and only if you are sensing person 
If you are activating radio or you were activating radio then stay there
# Patrol goals
If you are not activating carrying_item and you are not activating radio then visit dining
Always do person and do not person

