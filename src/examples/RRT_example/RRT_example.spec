# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
RRT with ODE

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
RRT_example.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p4
r5 = p3
r6 = p2
r1 = p6
others = p1

Spec: # Specification in structured English
#visit r2
visit r5
visit r4

