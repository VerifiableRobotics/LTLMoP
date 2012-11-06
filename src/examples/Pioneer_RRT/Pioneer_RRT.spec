# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
RRT with quadrotor

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
Pioneer_RRT.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p1
r3 = p2
r1 = p4
r2 = p3
others = 

Spec: # Specification in structured English
#robot starts in r1
visit r1
visit r3
visit r4

