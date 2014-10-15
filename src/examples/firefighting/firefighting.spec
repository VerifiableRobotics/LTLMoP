# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
drop, 1
radio, 1
extinguish, 0

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

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
deck = p7
porch = p3
dining = p6
bedroom = p8
others = 
kitchen = p5

Spec: # Specification in structured English
# Initial conditions
Env starts with false
Robot starts in porch with false

# Assumptions about the environment
If you were in porch then do not hazardous_item
If you were in porch then do not person

# Define robot safety including how to pick up
Do pick_up if and only if you are sensing hazardous_item and you are not activating carrying_item
If you are activating pick_up then stay there
carrying_item is set on pick_up and reset on drop
Do drop if and only if you are in porch and you are activating carrying_item

If you did not activate carrying_item then always not porch

# Define when and how to radio
Do radio if and only if you are sensing person
If you are activating radio or you were activating radio then stay there

# Patrol goals
Group rooms is living, bedroom, deck, kitchen, dining
If you are not activating carrying_item and you are not activating radio then visit all rooms
if you are activating carrying_item and you are not activating radio then visit porch

