# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
defuse, 1

CompileOptions:
convexify: False
parser: structured
fastslow: False
decompose: False
use_region_bit_encoding: True

CurrentConfigName:
pragbot

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
nowhere.converted.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bomb, 1
hostage, 1
badguy, 1
defuse_done, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Somewhere = Somewhere
Nowhere = Nowhere

Spec: # Specification in structured English
Visit Somewhere

