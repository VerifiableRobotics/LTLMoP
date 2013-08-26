# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: False
parser: structured
fastslow: False
decompose: False
use_region_bit_encoding: True

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
cardinal.converted.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
north = north
center = center
extra_east = extra_east
west = west
east = east
south = south

Spec: # Specification in structured English
Group rooms is north, south, extra_east
Visit all rooms

