# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
radio, 1
extinguish, 1

CompileOptions:
convexify: True
parser: ltl
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
Basic Simulation

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
floorplan.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
fire, 1
person, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
living = p4
deck = p7
porch = p3
dining = p6
bedroom = p8
others = 
kitchen = p5

Spec: # Specification in structured English
--
[]<>(deck)
[]<>(living)

