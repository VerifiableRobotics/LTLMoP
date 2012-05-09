# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
alarm, 1

CompileOptions:
convexify: False
fastslow: True

CurrentConfigName:
Saving our teammate

Customs: # List of custom propositions
search
check_in

RegionFile: # Relative path of region description file
bug_oldformat.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
animals, 1
emergency_alarm, 0
rescued, 0
get_gear, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
forest_2 = p7
forest_3 = p6
forest_1 = p8
forest_4 = p5
garage = p4
station = p2
others = p1

Spec: # Specification in structured English
# Pionner and Nao are on a mission to save one of their teammate in the forest. They have a vague idea
# about where he is. Their mission is to go and confirm the exact location by head-tap and then
# come back to the mission_station for help.

#Robot starts in garage
infinitely often not animals
group forest is forest_1,forest_2,forest_3,forest_4


visit station
visit all forest
if you are sensing animals then stay there
if you are sensing animals then do alarm

