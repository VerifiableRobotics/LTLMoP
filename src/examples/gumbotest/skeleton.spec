# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
defuse, 1
sweep, 1

CompileOptions:
convexify: False
parser: slurp
fastslow: False
decompose: False
use_region_bit_encoding: True

CurrentConfigName:
ros

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
nerve.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bomb, 1
sweep_done, 1
hostage, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
classroom1 = classroom1
east_hall = east_hall
classroom2 = classroom2
receiving = receiving
closet = closet
office = office
entrance = entrance
library = library
lounge = lounge
lab = lab
cafeteria = cafeteria
hall = hall
kitchen = kitchen

Spec: # Specification in structured English
Search the library, classroom1, and classroom2.
If you see a bomb, defuse it.
#If you see a hostage, stay there.

