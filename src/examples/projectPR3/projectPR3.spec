# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
ProjectPR3

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
..\CSharpRobot\demodayloop.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
exploreHack, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
r1 = p3
r2 = p2
r3 = p1

Spec: # Specification in structured English
Group regions is r1, r2, r3

visit r2

