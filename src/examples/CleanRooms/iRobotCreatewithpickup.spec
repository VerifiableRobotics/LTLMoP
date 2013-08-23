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
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
baiscSim

Customs: # List of custom propositions
carrying_marker
searchmode
retrievemode
confirmationmode

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
confirm, 1
abort, 1


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
environment starts with not confirm and not abort
robot starts with searchmode and not carrying_marker and not confirmationmode and not retrievemode and not pickup and not drop and not goToMarker

# search mode
searchmode is set on (retrievemode and drop) or (abort and confirmationmode) and reset on (markersNearBy and any rooms and not carrying_marker and not goToMarker and not abort )

if you are activating searchmode then visit all rooms
if you are activating searchmode and not carrying_marker then always not TrashCan

# confirmation mode
confirmationmode is set on (searchmode and markersNearBy and any rooms and not carrying_marker and not goToMarker and not abort ) and reset on abort or confirm
if you are activating confirmationmode then stay there
if you are sensing (abort and confirmationmode) then stay there

# retrieve mode
retrievemode is set on (confirmationmode and confirm) and reset on drop

do goToMarker if and only if you are activating retrievemode and you are not sensing hasArrived and you are not activating carrying_marker
do pickup if and only if you are activating retrievemode and you are sensing hasArrived  and you are not activating carrying_marker

do drop if and only if you are activating carrying_marker and you are in TrashCan
if you are activating carrying_marker then visit TrashCan
carrying_marker is set on pickup and reset on drop

