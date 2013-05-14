# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
tuck_arm, 1

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
RRT with ROS

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
Pioneer_RRT.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
flag, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p4
others = 
r1 = p1
r2 = p2
r3 = p3

Spec: # Specification in structured English
visit r1
visit r3
visit r4
if you are sensing flag then do tuck_arm

