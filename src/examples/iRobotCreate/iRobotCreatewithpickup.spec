# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
sing, 0
pickup, 1
drop, 1
blink, 0
goToMarker, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
new map

Customs: # List of custom propositions
carrying_marker
ready4Pickup
searchmode
approachingTarget

RegionFile: # Relative path of region description file
iRobotCreate.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bump_right, 0
bump_left, 0
wheelDrop_right, 0
wheelDrop_left, 0
wheelDrop_caster, 0
wall, 0
virtual_wall, 0
cliff_left, 0
cliff_right, 0
cliffFront_left, 0
cliffFront_right, 0
button_play, 0
button_advance, 0
markersNearBy, 1
hasArrived, 1


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

robot starts with searchmode and not ready4Pickup and not carrying_marker and not approachingTarget

searchmode is set on drop and reset on (markersNearBy and any rooms)
approachingTarget is set on (markersNearBy and any rooms) and reset on hasArrived
ready4Pickup is set on hasArrived and reset on pickup
carrying_marker is set on pickup and reset on drop


if you were activating carrying_marker then always not all room
if you are activating searchmode and you are not activating pickup then visit all rooms
visit TrashCan if and only if you are activating carrying_marker and you are not activating pickup
do goToMarker if and only if you are sensing (markersNearBy and any rooms) and you are not activating carrying_marker
do pickup if and only if you are activating ready4Pickup and you are not activating carrying_marker
do drop if and only if you are activating carrying_marker and you are in TrashCan

