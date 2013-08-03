# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
isOpen, 1

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
projectPR3_lab.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
door_closed, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
hallway = p6, p7, p8, p9, p10
office = p1
others = 
doorstep = p5

Spec: # Specification in structured English
robot starts in hallway

do isOpen if and only if you are not sensing door_closed

if you are in doorstep and you are sensing door_closed then stay there

infinitely often not door_closed

go to office

