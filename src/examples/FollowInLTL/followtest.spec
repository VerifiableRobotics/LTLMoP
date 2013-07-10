# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
beep, 0

CompileOptions:
convexify: True
parser: ltl
fastslow: False
decompose: True
use_region_bit_encoding: True

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
deck = p7
porch = p3
dining = p6
bedroom = p8
others = 
kitchen = p5

Spec: # Specification in structured English

# === env assumptions ===
# basically, to keep the automaton smaller,
# we assume that the thing we are following will follow
# the topology of the map the same way we must,
# and shall not skip between rooms

# this part is auto-generated because it is messy

FOLLOW_SENSOR_CONSTRAINTS

--

# === sys reqs ===
# start in the porch
porch

# keep track of whether the target moved during its last "turn" or not
[](next(s.env_stationary) = ((e.sbit0 = next(e.sbit0)) & (e.sbit1 = next(e.sbit1)) & (e.sbit2 = next(e.sbit2))))

# always end up in the same place as the target,
# although weakened to let us stay in place if
# the target is constantly moving (it is not clear
# what "follow" would even mean in this scenario, for that matter)

# single-goal style: []<>(s.env_stationary -> ((s.bit0 = e.sbit0) & (s.bit1 = e.sbit1) & (s.bit2 = e.sbit2)))

# multi-goal style:
[]<>((s.env_stationary & (!e.sbit0 & !e.sbit1 & !e.sbit2)) -> (!s.bit0 & !s.bit1 & !s.bit2))
[]<>((s.env_stationary & (!e.sbit0 & !e.sbit1 & e.sbit2)) -> (!s.bit0 & !s.bit1 & s.bit2))
[]<>((s.env_stationary & (!e.sbit0 & e.sbit1 & !e.sbit2)) -> (!s.bit0 & s.bit1 & !s.bit2))
[]<>((s.env_stationary & (!e.sbit0 & e.sbit1 & e.sbit2)) -> (!s.bit0 & s.bit1 & s.bit2))
[]<>((s.env_stationary & (e.sbit0 & !e.sbit1 & !e.sbit2)) -> (s.bit0 & !s.bit1 & !s.bit2))
[]<>((s.env_stationary & (e.sbit0 & !e.sbit1 & e.sbit2)) -> (s.bit0 & !s.bit1 & s.bit2))
[]<>((s.env_stationary & (e.sbit0 & e.sbit1 & !e.sbit2)) -> (s.bit0 & s.bit1 & !s.bit2))
[]<>((s.env_stationary & (e.sbit0 & e.sbit1 & e.sbit2)) -> (s.bit0 & s.bit1 & s.bit2))


# UNREALIZABILITY TEST:
# never go in the bedroom!
#[](!bedroom)

