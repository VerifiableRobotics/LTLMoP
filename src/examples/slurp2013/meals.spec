# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
drop, 1

CompileOptions:
convexify: True
parser: slurp
fastslow: False
decompose: True

CurrentConfigName:
basicsim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
slurp_hospital.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

Spec: # Specification in structured English
# Example of unsatisfiability, and state-wise goal-local "optimal" planning.
# Note: The robot can carry up to 2 meals at a time (this is somehow implicit...?)

Don't take meals into any public area.
Deliver meals from the kitchen to each patient room.

