# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
T_narrow, 1
T_low, 1
T_hardware, 1
T_stationary, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
volcano examp

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
volcano.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
Landslide, 1
Burn, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Lava3 = p6
Lava2 = p7
Lava1 = p8
others = 
between$Lava1$and$Lava2$ = p1, p2
Plains = p5
Plateau = p4
between$Plains$and$Springs$ = p2, p8
Springs = p3

Spec: # Specification in structured English
# Volcano Example
# Visit Plateu and Plains repeatedly as long as there is no landslide, while avoiding lava areas.
# If there is a landslide, then stay put.  If burning, then visit Springs.

Env starts with false
Robot starts in Plains
# Always do T_hardware
Always T_hardware
If you are not sensing Landslide and you are not sensing Burn then visit Plateau
If you are not sensing Landslide and you are not sensing Burn then visit Plains
If you are not sensing Landslide and you are sensing Burn then visit Springs
If you were in between Plains and Springs then do not Springs
Do T_stationary if and only if you are in Springs and you are sensing Burn
Always not Lava1 and not Lava2 and not Lava3
# If you were in between Lava1 and Lava2 then do not sense Landslide
If you were in between Lava1 and Lava2 then do not Landslide
T_low is set on between Lava1 and Lava2 and reset on Plains or Plateau or Springs
T_narrow is set on between Lava1 and Lava2 and reset on Plains or Plateau or Springs
If you are sensing Landslide then stay there

