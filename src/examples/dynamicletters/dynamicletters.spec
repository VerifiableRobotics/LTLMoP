# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
deliver_letter1, 1
resynthesize, 1

CompileOptions:
convexify: True
parser: nltk
fastslow: False
decompose: True
use_region_bit_encoding: True

Customs: # List of custom propositions
carrying_letter1

RegionFile: # Relative path of region description file
slurp_hospital.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
new_letter, 1
letter1, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p3
r5 = p2
r6 = p1
r1 = p6
closet = p12
r3 = p4
hall_w = p13, p14
mail_room = p7
r2 = p5
lounge = p8
hall_n = p15, p16
others = 
hall_c = p11

Spec: # Specification in structured English
#### Group definitions ####

# Letters that we can detect
Group letters is letter1.

# Propositions indicating that we are currently carrying a given letter
Group letter_slots is carrying_letter1.

# Regions that each letter needs to end up in.
Group letter_destinations is r1.

# Actions that deliver each letter
Group letter_delivery is deliver_letter1.

#### Spec-rewriting and resynthesis mechanics ####

# This is only triggered if we see a letter we haven't ever seen before
If you are sensing new_letter then add to group letters and resynthesize.

#### Letter delivery specification ####

# Keep track of what we're carrying
Corresponding letter_slot is set on mail_room and sensing any letter and pick_up and reset on corresponding letter_delivery.

# Deliver only in the destination room, and only if you have that letter
If you are not in the corresponding letter_destination and activating the corresponding letter_slot then do not do any letter_delivery.

# No spurious pickups
If you are not sensing any letter then do not do pick_up.

# Our goal is to get rid of all our letters
Infinitely often not any letter_slots.

# Go back to the mailroom if we have nothing else to do
If you are not activating any letter_slots then go to mail_room.

