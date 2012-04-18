# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
hmwk_given, 0
hmwk_taken, 0
wait_until_given, 0
wait_until_taken, 0

CurrentConfigName:
bug ode

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
Feb21.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
homework, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = p1
r1 = p4
r2 = p2

Spec: # Specification in structured English
#Env starts with false
#Robot starts in lab
#Robot starts with false

#Always not obs

Visit r1
Visit r2

