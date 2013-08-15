# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
isOpen, 0

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
ProjectPR3_debug

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
projectPR3.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
open_doorway, 0
door_closed, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
#if you are not sensing door_closed then visit r2 and stay there

visit r1
visit r2
#do isOpen if and only if you are not sensing door_closed

#if you are sensing door_closed or you were sensing door_closed then stay there

