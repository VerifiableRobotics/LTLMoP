# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
sweep, 1
defuse, 1
call, 1

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
bomb, 1
hostage, 1
sweep_done, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p4
r5 = p3
r6 = p2
r1 = p7
closet = p10
r3 = p5
lounge = p8
r2 = p6
others = p11, p12, p13, p14, p15, p16, p17, p18, p19
kitchen = p9

Spec: # Specification in structured English
If you see a bomb, go to the lounge.

