# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 1 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0147636360743,-8.0510471283,-0.0164705884223,6.03491129127

InitialRegion: # Initial region number
1

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
cornell_asl

Name: # Name of the experiment
ASL

RobotFile: # Relative path of robot description file
pioneer_real.robot


======== EXPERIMENT CONFIG 2 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0145090906457,-7.97493804517,-0.0163607845119,5.97177404282

InitialRegion: # Initial region number
1

InitialTruths: # List of initially true propositions

Lab: # Lab configuration file
playerstage

Name: # Name of the experiment
PlayerStage

RobotFile: # Relative path of robot description file
pioneer_stage.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
pick_up,1
drop,1
radio,1
extinguish,1

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
icra10.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
fire,1
person,1
hazardous_item,1

currentExperimentName:
PlayerStage


======== SPECIFICATION ========

Spec: # Specification in simple English
### Initial Conditions ###

Env starts with false
Robot starts with false

### Transitions ###

# Find and remove to the outdoors potentially hazardous items
Do pick_up if and only if you are sensing hazardous_item and you are not activating carrying_item
Do drop if and only if you are activating carrying_item and you were in porch
If you are activating pick_up or you activated pick_up then stay there
If you are activating drop or you activated drop then stay there

# Define the logic of picking up, carrying, and dropping an item
If you activated pick_up then do carrying_item
If you activated drop then do not carrying_item
If you activated carrying_item and you did not activate drop then do carrying_item
If you did not activate carrying_item and you did not activate pick_up then do not carrying_item

If you were in porch then do not hazardous_item
If you did not activate carrying_item then always not porch

# Call for help if you encounter a person
Do radio if and only if you are sensing person
If you are activating radio or you activated radio then stay there
If you were in porch then do not person

# Extinguish fires as if you were a firefighting robot
Do extinguish if and only if you are sensing fire
If you are activating extinguish or you activated extinguish then stay there
If you were in porch then do not fire

### Goals ###

If you are not activating carrying_item and you are not activating radio and you are not activating extinguish then visit deck
If you are not activating carrying_item and you are not activating radio and you are not activating extinguish then visit living
If you are not activating carrying_item and you are not activating radio and you are not activating extinguish then visit bedroom
If you are not activating carrying_item and you are not activating radio and you are not activating extinguish then visit dining
If you are not activating carrying_item and you are not activating radio and you are not activating extinguish then visit kitchen
If you are activating carrying_item and you are not activating radio and you are not activating extinguish then visit porch

