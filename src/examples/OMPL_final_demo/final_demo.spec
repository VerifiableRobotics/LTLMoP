# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
OMPL with quadrotor

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
final_demo.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r6 = p3
r7 = p2
r1 = p8
r2 = p7
r3 = p6
others = 
r8 = p1

Spec: # Specification in structured English
visit r1
visit r3
visit r8
visit r6
visit r7

