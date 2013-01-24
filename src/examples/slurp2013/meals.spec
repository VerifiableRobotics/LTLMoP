# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
drop, 1
pickup, 1

CompileOptions:
convexify: True
parser: slurp
fastslow: False
decompose: True

CurrentConfigName:
basicsim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
slurp_hospital.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p3
r5 = p2
r6 = p1
r1 = p6
closet = p12
r3 = p4
hall_W = p13, p14
lounge = p7
r2 = p5
hall_N = p15, p16
others = 
hall_C = p11
kitchen = p8

Spec: # Specification in structured English
# Example of unsatisfiability, and state-wise goal-local "optimal" planning.
# Note: The robot can carry up to 2 meals at a time (this is somehow implicit...?)

Start in the lounge.
Carry meals from the kitchen to r5.

#The meals are in the kitchen.
#Carry meals to all patient rooms.
#Don't go to all public rooms.

