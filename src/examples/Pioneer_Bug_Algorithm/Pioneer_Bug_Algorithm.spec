# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
test

Customs: # List of custom propositions
search
check_in

RegionFile: # Relative path of region description file
Pioneer_Bug_Algorithm.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
animals, 1
emergency_alarm, 1
rescued, 1
get_gear, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
forest_2 = p7
forest_3 = p6
forest_1 = p8
forest_4 = p5
garage = p4
station = p2
others = p9, p10, p11, p12, p13

Spec: # Specification in structured English
# Pionner and Nao are on a mission to save one of their teammate in the forest. They have a vague idea
# about where he is. Their mission is to go and confirm the exact location by dropping a flag and then
# come back to the mission_station for help.

Robot starts in garage
infinitely often not animals
group forest is forest_1,forest_2,forest_3,forest_4

check_in is set on (garage and emergency_alarm) and reset on (station and get_gear)
search is set on (station and get_gear) and reset on (rescued and any forest)


If you are not activating (check_in or search) and you are not sensing animals then go to garage and stay there

Visit station if and only if you are activating check_in and you are not sensing animals

#if you are activating check_in and you are not sensing animals then visit station

Visit all forest if and only if you are activating search and you are not sensing animals

If you are sensing animals then stay there

