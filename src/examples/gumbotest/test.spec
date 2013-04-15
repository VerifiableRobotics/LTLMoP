# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
panic, 1

CompileOptions:
convexify: False
parser: structured
fastslow: False
decompose: False
use_region_bit_encoding: True

CurrentConfigName:
ros

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
4thfloor.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
badguy, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
classroom = classroom
office = office
library = library
lounge = lounge
lab = lab
hall = hall
kitchen = kitchen

Spec: # Specification in structured English
group cool_places is classroom
do panic if and only if you are sensing badguy
if you are not sensing badguy then visit all cool_places

if you are sensing start of badguy or end of badguy then stay there

