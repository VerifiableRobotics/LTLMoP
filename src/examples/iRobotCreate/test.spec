# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
drop, 1
driveToMarker, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
newmap

Customs: # List of custom propositions
search
goToMarker
ready4Pickup
carry

RegionFile: # Relative path of region description file
iRobotCreate.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
see, 1
arrive, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Bathroom = p8
LivingRoom = p4
MasterBedroom = p3
DiningRoom = p6
Bedroom = p7
TrashCan = p2
others = p9, p10, p11
Kitchen = p5

Spec: # Specification in structured English
group rooms is Bedroom,Kitchen,DiningRoom,LivingRoom,MasterBedroom,Bathroom

robot starts with search

search is set on drop and reset on (see and any rooms)
goToMarker is set on (see and any rooms) and reset on arrive
ready4Pickup is set on not goToMarker and reset on pickup
carry is set on pickup and reset on drop


if you were activating carry then always not all room
if you are activating search then visit all rooms
visit TrashCan if and only if you are activating carry
do driveToMarker if and only if you are activating goToMarker
do pickup if and only if you are activating ready4Pickup
do drop if and only if you are activating carry and you are in TrashCan

