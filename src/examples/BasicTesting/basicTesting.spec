# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
x, 1
y, 1
z, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
basicTesting.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
a, 1
b, 1
c, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p3
r2 = p2
others = p4, p5, p6, p7, p8, p9

Spec: # Specification in structured English
robot starts in r1 with false
infinitely often not a and not b and not c

visit r2
visit r1

if you are sensing a then stay there
if you are sensing b then stay there
if you are sensing c then stay there

do x if and only if a
do y if and only if b
do z if and only if c

