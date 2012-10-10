# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
T_stationary, 1
T_fast, 1
T_low, 1
T_nonholonomic_turning, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Test1

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
predator.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
predator, 1
prey, 1
poison, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Mountain2 = p13, p14
Mountain = p15
Bridge = p22, p23
Field = p19, p20
Tunnel = p10, p11
Island = p17, p18
Water3 = p4
Water2 = p5, p6, p7, p8
between$Island$and$Dock$ = p7, p8, p22, p23
Water = p9
Dock = p21
others = p2, p24, p25
Springs = p12
Meadows = p16
between$Meadows$and$Dock$ = p3, p6, p8, p11, p14, p15, p18, p20, p23

Spec: # Specification in structured English
# This example simulates a wild animal who moves between several regions, hunts the preys and avoids the predators.
# It does certain actions in certain regions, for example, like automatically going to Springs when it senses poison.


Env starts with false
Robot starts in Island
Always not Water and not Water2 and not Water3

If you were in Tunnel then always not prey
If you were in between Island and Dock then always not prey

If you are not sensing predator and you are not sensing prey and you are not sensing poison then visit Meadows
If you are not sensing predator and you are not sensing prey and you are not sensing poison then visit Dock
If you are not sensing predator and you are not sensing prey and you are sensing poison then visit Springs

Do T_low and T_nonholonomic_turning if and only if you are in Tunnel

T_fast is set on between Island and Dock and reset on Dock
If you are sensing predator then stay there
If you are sensing prey and you are in Dock or Field or Meadows or Springs then do T_stationary

