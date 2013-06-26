# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
turnLeft, 1
turnRight, 1
dontMove, 1

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
basicSim

Customs: # List of custom propositions
left
right
crossedBlackLeft
crossedBlackRight
stopEverything

RegionFile: # Relative path of region description file
linefollower.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
white, 1
black, 1
touch, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
robot starts in r1 with left and not right and not crossedBlackLeft and not crossedBlackRight and not stopEverything
#always (white and not black) or (black and not white)
infinitely often touch

always r1

left is set on white and crossedBlackRight and reset on crossedBlackLeft and white
right is set on white and crossedBlackLeft and reset on crossedBlackRight and white

do turnLeft if and only if left and not right
do turnRight if and only if right and not left

crossedBlackLeft is set on black and left and reset on white and left
crossedBlackRight is set on black and right and reset on white and right

stopEverything is set on touch and not stopEverything and reset on touch and stopEverything

if stopEverything then do dontMove

