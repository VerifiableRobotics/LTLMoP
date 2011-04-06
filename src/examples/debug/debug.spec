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
pick_up,1
drop,1
radio,1
extinguish,0

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
debug.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
fire,0
person,1
hazardous_item,1

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
Env starts with false
Robot starts  with false
#If you activated drop then do not drop
#If activated drop then do drop
#If you did not activate drop then do drop
Do pick_up if and only if you are sensing hazardous_item
Do drop if and only if you were in porch
If you are activating pick_up or you activated pick_up then stay there
If you are activating drop or you activated drop then stay there
If you did not activate pick_up then always not porch
Visit porch
#Always person and not person














#NON-TRIVIAL EXAMPLES:

#Environment liveness unrealizable
#Env starts with false
#Robot starts with false
#robot starts with false
#If you were in porch then do not hazardous_item
#If drop then not drop
#If drop then drop
#If not drop then drop
#Do pick_up if and only if you are sensing hazardous_item
#Do drop if and only if you were in porch
#If you are activating pick_up or you activated pick_up then go to porch and stay there
#If you are activating drop or you activated drop then stay there
#If you did not activate pick_up then always not porch
#Visit porch
#If you were in porch then infinitely often person and not person
#Go to porch and stay there

##System safety unrealizable
#Env starts with false
#Robot starts in porch
#If you were in porch then do not hazardous_item
#Do pick_up if and only if you are sensing hazardous_item
#Do drop if and only if you were in porch
#If you are activating pick_up or you activated pick_up then stay there
#If you are activating drop or you activated drop then stay there
#If you did not activate pick_up then always not porch
#Visit porch
##If you were in porch then infinitely often person and not person



#SIMPLE EXAMPLES OF EACH CASE:

##System liveness unrealizable
#Env starts with false
#Robot starts in deck
#Visit porch
#If you are sensing person then do not porch
#Visit person

##Environment liveness unrealizable
#Env starts with false
#Robot starts in deck
#Visit porch
#If you are sensing person then do not porch
#Visit not person
#If you were in deck then do person

##System safety unrealizable
#Env starts with false
#Robot starts in deck
#Always not porch
#If you are sensing person then do porch

##Environment safety unrealizable
#Env starts with false
#Robot starts in deck
#Always not person
#If you were in porch then do  person

##System liveness and safety unsatisfiable
#Env starts with false
#Robot starts in deck
#Always not porch
#Infinitely often porch

##Environment liveness and safety unsatisfiable
#Env starts with false
#Robot starts in deck
#Always not person
#Infinitely often person

##System safety unsatisfiable
#Env starts with false
#Robot starts in deck
#Always porch

##Environment safety unsatisfiable
#Env starts with false
#Robot starts in deck
#Always person and not person


