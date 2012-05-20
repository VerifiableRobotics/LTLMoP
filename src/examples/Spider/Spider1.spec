# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
setInclineGaits, 1
setGravelGaits, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
SpiderFieldWorking.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
naotap, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
slope = p5
r1 = p6
mesh = p7
others = 
z4 = p1
z1 = p4
z2 = p3
z3 = p2

Spec: # Specification in structured English
do setGravelGaits if and only if you are in mesh
do setInclineGaits if and only if you are in slope

visit z1
visit z2
visit z3
visit z4

