# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
hmwk_given, 0
hmwk_taken, 0
wait_until_given, 0
wait_until_taken, 0
stop, 0

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
RRT with Pioneer

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
Feb21.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
see_dynamic_obstacle, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p3
r5 = p2
r1 = p6
r2 = p5
r3 = p4
others = p1

Spec: # Specification in structured English
#Env starts with false
#Robot starts in lab
#Robot starts with false

#Always not obs

#do stop if and only if you are sensing see_dynamic_obstacle
#if you are sensing  see_dynamic_obstacle then stay there
visit r3
visit r4
visit r5

