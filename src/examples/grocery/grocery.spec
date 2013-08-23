# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
look_leftright, 1
call_manager, 1
sit_down, 1
say_impossible, 1
say_spill, 1

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
basicSim

Customs: # List of custom propositions
spill_top
spill_bottom

RegionFile: # Relative path of region description file
grocery.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
see_spill, 1
see_missingitem, 1
head_tapped, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p9
office = p13
between$r3$and$r4$ = p14
r3 = p10
between$r1$and$r2$ = p15
r2 = p11
others = p1, p2, p3, p4, p5
r1 = p12

Spec: # Specification in structured English
group Corners is r1, r2, r3, r4
robot starts in any Corner with false

# Remember spills you've detected
spill_top is set on (r1 or r2) and see_spill and reset on false
spill_bottom is set on (r3 or r4) and see_spill and reset on false

# Avoid aisles with spills
if you are activating spill_top then always not between r1 and r2
if you are activating spill_bottom then always not between r3 and r4

# Call the manager when you find a missing item
call_manager is set on see_missingitem and reset on head_tapped
#if you are activating call_manager then stay there
if start of call_manager then stay there
if you are activating call_manager and you are not activating spill_top and spill_bottom then visit office

if you are sensing head_tapped then stay there
infinitely often not head_tapped

if start of spill_bottom then stay there
if start of spill_top then stay there

# Patrol the store
if you are not activating call_manager and you are not activating spill_top and spill_bottom then visit all Corners
#if you are not activating spill_top and spill_bottom then visit all Corners
do look_leftright if and only if you are not activating call_manager and you were in (between r1 and r2 or between r3 and r4)

do say_impossible if and only if you are activating spill_top and spill_bottom
do sit_down if and only if you are activating spill_top and spill_bottom
if you are activating sit_down then stay there

do say_spill if and only if start of spill_top and you are not activating say_impossible or start of spill_bottom and you are not activating say_impossible

