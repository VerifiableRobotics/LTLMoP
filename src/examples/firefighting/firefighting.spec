# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
drop, 1
radio, 1
extinguish, 0

CompileOptions:
convexify: False
parser: slurp
fastslow: False
decompose: False
use_region_bit_encoding: True

CurrentConfigName:
Basic Simulation

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
floorplan.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
fire, 0
person, 1
hazardous_item, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
living = p4
porch = p3
deck = p7
others = 
dining = p6
bedroom = p8
kitchen = p5

Spec: # Specification in structured English


