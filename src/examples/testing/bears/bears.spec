# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
drop, 1
play_dead, 1
sing, 1

CurrentConfigName:
basicsim

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
bears.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bear, 1
fruit, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
bridge = p9
between$bridge$and$apple_tree$ = p10, p55, p56, p57
bear_colony = p13
near$bear_colony$40 = p13, p50, p51, p52, p53, p54
pear_tree = p4
apple_tree = p14
lemon_tree = p7
others = p15, p16, p17, p18, p19, p20, p21, p22, p25, p26, p27, p28, p29, p30, p31, p32, p33, p34, p35, p36, p37, p38, p39, p40, p41, p42, p43, p44, p45, p46, p47
cabin = p8

Spec: # Specification in structured English
# This examples demonstrates the use of a few locative prepositions

Env starts with false
Robot starts in cabin with false

Always not within 40 of bear_colony

If you were in cabin then do not bear
If you were in cabin then do not fruit

carrying_item is set on pick_up and reset on drop
If you are activating pick_up or you activated pick_up then stay there
If you are activating drop or you activated drop then stay there

Do pick_up if and only if you are sensing fruit and you are not activating carrying_item
Do drop if and only if you are activating carrying_item and you were in cabin

Do play_dead if and only if you are sensing bear
If you are activating play_dead or you activated play_dead then stay there

Do sing if and only if you are in between bridge and apple_tree

Group trees is pear_tree, lemon_tree, apple_tree
If you are not activating carrying_item and you are not activating play_dead then visit all trees

If you did not activate carrying_item then always not cabin
If you are activating carrying_item and you are not activating play_dead then visit cabin

