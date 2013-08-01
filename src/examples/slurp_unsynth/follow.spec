# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: False
parser: slurp
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
basicsim_follow

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
slurp_hospital.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
sbit0, 1
sbit1, 1
sbit2, 1
sbit3, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p3
r5 = p2
r6 = p1
r1 = p6
closet = p12
r3 = p4
hall_w = p9
lounge = p7
r2 = p5
hall_n = p10
others = 
hall_c = p11
kitchen = p8

Spec: # Specification in structured English
# An example of unrealizability, and powerful language constructs.

Follow me.
Don't go into the kitchen.

