# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
inASL, 1

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
OMPL with quadrotor

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
final_demo.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
present_delivered, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Rovaniemi = p5
Cornell = p3
NYC = p4
Atlantic_Ocean = p2
ASL = p1
others = 

Spec: # Specification in structured English
Robot starts in Rovaniemi with false

inASL is set on ASL and reset on false

if you are not activating inASL then visit Atlantic_Ocean
if you are not activating inASL then visit NYC
if you are not activating inASL then visit Cornell
if you are not activating inASL then visit ASL

if you are sensing present_delivered then go to Rovaniemi

