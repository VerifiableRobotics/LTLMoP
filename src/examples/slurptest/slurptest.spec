# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 0
drop, 0
radio, 1
extinguish, 0
sweep, 1

CompileOptions:
convexify: True
parser: slurp
fastslow: False
decompose: True

CurrentConfigName:
basicsim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
iros10.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
fire, 0
person, 0
hazardous_item, 0
sweep_done, 1


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
Search the porch and the deck.
Don't go to the kitchen.

