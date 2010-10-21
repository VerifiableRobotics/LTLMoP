# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.
# Note that all values are separated by *tabs*.


======== EXPERIMENT CONFIG 0 ========

Calibration: # Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset
0.0166959456495,-8.03013490342,-0.0199230771798,6.05661546267

InitialRegion: # Initial region number
3

InitialTruths: # List of initially true propositions
pick_up
drop
play_dead
sing
carrying_item
bear
fruit

Lab: # Lab configuration file
playerstage.lab

Name: # Name of the experiment
test1

RobotFile: # Relative path of robot description file
pioneer_stage.robot


======== SETTINGS ========

Actions: # List of actions and their state (enabled = 1, disabled = 0)
pick_up,1
drop,1
play_dead,1
sing,1

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
lpdemo.regions

Sensors: # List of sensors and their state (enabled = 1, disabled = 0)
bear,1
fruit,1

currentExperimentName:
test1


======== SPECIFICATION ========

RegionMapping:

bridge=p9
between$bridge$and$apple_tree$=p10,p12,p54,p55,p56
near$bear_colony$40=p13,p49,p50,p51,p52,p53
bear_colony=p13
pear_tree=p4
upper_river=p12,p23,p24
apple_tree=p14
lemon_tree=p7
others=p15,p16,p17,p18,p19,p20,p21,p22,p25,p26,p27,p28,p29,p30,p31,p32,p33,p34,p35,p36,p37,p38,p39,p40,p41,p42,p43,p44,p45,p46
lower_river=p47,p48
cabin=p8

Spec: # Specification in simple English
Env starts with false
Robot starts in cabin
Always not lower_river
Always not upper_river
Always not within 40 of bear_colony
If you were in cabin then do not bear
If you were in cabin then do not fruit
Do pick_up if and only if you are sensing fruit and you are not activating carrying_item
Do drop if and only if you are activating carrying_item and you were in cabin
If you are activating pick_up or you activated pick_up then stay there
If you are activating drop or you activated drop then stay there
Do play_dead if and only if you are sensing bear
If you are activating play_dead or you activated play_dead then stay there
Do sing if and only if you are in between bridge and apple_tree
If you activated pick_up then do carrying_item
If you activated drop then do not carrying_item
If you activated carrying_item and you did not activate drop then do carrying_item
If you did not activate carrying_item and you did not activate pick_up then do not carrying_item
If you are not activating carrying_item and you are not activating play_dead  then visit pear_tree
If you are not activating carrying_item and you are not activating play_dead  then visit lemon_tree
If you are not activating carrying_item and you are not activating play_dead  then visit apple_tree
If you did not activate carrying_item then always not cabin
If you are activating carrying_item and you are not activating play_dead then visit cabin

