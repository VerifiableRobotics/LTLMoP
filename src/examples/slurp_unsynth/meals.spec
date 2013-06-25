# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
drop, 1
pickup, 1
camera, 0

CompileOptions:
convexify: True
parser: slurp
fastslow: False
decompose: True
use_region_bit_encoding: False

CurrentConfigName:
basicsim_meals

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
hall_w = p13, p14
lounge = p7
r2 = p5
hall_n = p15, p16
others = 
hall_c = p11
kitchen = p8

Spec: # Specification in structured English
# Unsat livelock
Carry meals from the kitchen to all patient rooms.
Don't go into any public rooms.

