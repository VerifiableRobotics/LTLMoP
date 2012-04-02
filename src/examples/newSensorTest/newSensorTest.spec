# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
radio, 1
pick_up, 0
drop, 0

CurrentConfigName:
Experiment with player stage

Customs: # List of custom propositions
imacustom

RegionFile: # Relative path of region description file
newSensorTest.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
hazardous_item, 0
person, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
r1 = p3
r2 = p2
r3 = p1

Spec: # Specification in structured English
visit r1
visit r2
visit r3
do radio if and only if you are sensing person

