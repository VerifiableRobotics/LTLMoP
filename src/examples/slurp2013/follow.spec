# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

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
alarm, 1
sbit0, 1
sbit1, 1
sbit2, 1
sbit3, 1


======== SPECIFICATION ========

Spec: # Specification in structured English
# An example of unrealizability, and powerful language constructs.

Follow me.

If you hear an alarm, avoid the kitchen.
# ^^ Can we come up with a more interesting avoid statement?

