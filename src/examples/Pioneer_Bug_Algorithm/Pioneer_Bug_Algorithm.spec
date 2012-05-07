# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
alarm, 1

CompileOptions:
convexify: True
fastslow: True

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
# infinitely often not animals
infinitely often not emergency_alarm
group forest is forest_1,forest_2,forest_3,forest_4

#standby is set on (rescued and  any forest) and reset on  (garage and emergency_alarm)
check_in is set on (garage and emergency_alarm) and reset on (station and get_gear)
#if you are sensing emergency_alarm and you are in garage then stay there
search is set on (station and get_gear) and reset on (rescued and any forest)


if you are sensing emergency_alarm and you are in garage then stay there

#Visit station if and only if you are activating check_in

#if you are activating check_in then visit station

#if you are activating search then visit all forest
visit all forest if and only if you are activating search

#If you are sensing animals then stay there
do alarm if and only if you are sensing animals

#If you are activating standby then go to garage

