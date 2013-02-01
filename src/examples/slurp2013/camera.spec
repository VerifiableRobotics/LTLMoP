# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
drop, 0
pickup, 0
camera, 1

CompileOptions:
convexify: False
parser: slurp
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
basicsim_camera

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
hall_w = p9
lounge = p7
r2 = p5
hall_n = p10
others = 
hall_c = p11
kitchen = p8

Spec: # Specification in structured English
# Unsat deadlock
Don't activate your camera in any restricted area.
Avoid the lounge.

Start in hall_c.
Always activate your camera.

