# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
drop, 1

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True

CurrentConfigName:
simulation

Customs: # List of custom propositions
left_full
right_full
delivered_r1
delivered_r2

RegionFile: # Relative path of region description file
meals.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16
r1 = p3
r2 = p2
kitchen = p4

Spec: # Specification in structured English
robot starts with false

delivered_r1 is set on r1 and drop and reset on false
delivered_r2 is set on r2 and drop and reset on false

infinitely often delivered_r1 and delivered_r2

if you are not activating right_full or left_full then do not drop
if you are activating right_full and left_full then do not pickup

# give priority to the left side for both pickup and drop
left_full is set on pickup and not left_full and reset on drop and left_full
right_full is set on pickup and left_full and reset on drop and not left_full

if you were activating pickup then stay there

if you are not in kitchen then do not pickup

