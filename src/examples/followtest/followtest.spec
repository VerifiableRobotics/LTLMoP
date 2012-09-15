# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
beep, 0

CompileOptions:
convexify: True
parser: ltl
fastslow: False

CurrentConfigName:
basicsim

Customs: # List of custom propositions
env_stationary

RegionFile: # Relative path of region description file
../firefighting/floorplan.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
sbit0, 1
sbit1, 1
sbit2, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
living = p4
porch = p3
deck = p7
others = 
dining = p6
bedroom = p8
kitchen = p5

Spec: # Specification in structured English

# env assumptions

ENVIRONMENT TOPOLOGY

#things are OK if we force the env to always stay in r6
#(!e.sbit0 & !e.sbit1 & !e.sbit2 & !e.sbit3 & !e.sbit4)
#[](!next(e.sbit0) & !next(e.sbit1) & !next(e.sbit2) & !next(e.sbit3) & !next(e.sbit4))
--

# sys reqs
porch
#[](next(s.env_stationary) = ((e.sbit0 = next(e.sbit0)) & (e.sbit1 = next(e.sbit1)) & (e.sbit2 = next(e.sbit2)) & (e.sbit3 = next(e.sbit3)) & (e.sbit4 = next(e.sbit4))))
[](next(s.env_stationary) = ((e.sbit0 = next(e.sbit0)) & (e.sbit1 = next(e.sbit1)) & (e.sbit2 = next(e.sbit2))))

#[]<>(s.env_stationary -> ((s.bit0 = e.sbit0) & (s.bit1 = e.sbit1) & (s.bit2 = e.sbit2) & (s.bit3 = e.sbit3) & (s.bit4 = e.sbit4)))
[]<>(s.env_stationary -> ((s.bit0 = e.sbit0) & (s.bit1 = e.sbit1) & (s.bit2 = e.sbit2)))

# beep if env is in r6
#[](next(s.beep) <-> (!next(e.sbit0) & !next(e.sbit1) & !next(e.sbit2) & !next(e.sbit3) & !next(e.sbit4)))

