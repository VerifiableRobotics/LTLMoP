# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
knock, 1

CompileOptions:
convexify: False
parser: structured
fastslow: True
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
ProjectPR3_lab

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
projectPR3_lab.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
door_closed, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
hallway = p2
office = p1
others = 
doorstep = p3

Spec: # Specification in structured English
robot starts in hallway with false
env starts with false

#isOpen is set on (not door_closed) and reset on (door_closed and doorstep)

if you were in doorstep and you are sensing door_closed then do not office
if you were in doorstep and you are sensing door_closed then do knock

infinitely often not door_closed

go to office

