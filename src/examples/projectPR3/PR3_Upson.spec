# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
Pickup_Letter, 1
Deliver_Letter, 1
Knock, 1
Return_Letter, 1

CompileOptions:
convexify: False
parser: nltk
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
ProjectPR3_upson

Customs: # List of custom propositions
Carrying_Letter_KressGazit
Delivering
Empty_Office
Locked_Office

RegionFile: # Relative path of region description file
PR3.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
Letter_KressGazit, 1
Door_Closed, 1
Person, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Classroom = p7
Mail_Room = p6
others = p1
Office_Campbell = p5
Office_KressGazit = p4

Spec: # Specification in structured English
# Init
Robot starts  with false
Environment starts with false

# Set/Reset the indicator for whether or not the robot is carrying a letter
Carrying_Letter_KressGazit is set on Pickup_Letter and reset on Deliver_Letter or Return_Letter

# Set/Reset the mode for whether the robot is delivering a letter, or returning it to the mail room
Do Locked_Office if and only if you activated Knock and you are sensing Door_Closed
Do Empty_Office if and only if you are not sensing Person and you are not sensing Door_Closed and you are in Office_KressGazit
Delivering is set on Pickup_Letter and reset on Deliver_Letter or Locked_Office or Empty_Office

# Go to the mail room to pick up any letters, or to drop off extras
If you are not activating Carrying_Letter_KressGazit then go to Mail_Room
If you are not activating Delivering then go to Mail_Room
#If you are not activating Delivering and you are activating Carrying_Letter_KressGazit and you are in Mail_Room then do Deliver_Letter
Do Return_Letter if and only if you are not activating Delivering and you are activating Carrying_Letter_KressGazit and you are in Mail_Room

# When in the mail room, pick up any letters
#If you are in Mail_Room and you are sensing Letter_KressGazit then do Pickup_Letter
Do Pickup_Letter if and only if you are in Mail_Room and you are sensing Letter_KressGazit

# Deliver any letters to their appropriate offices, if someone is there to take the letter
If you are activating Carrying_Letter_KressGazit and you are activating Delivering then go to Office_KressGazit
#If you are activating Carrying_Letter_KressGazit and you are sensing Door_Closed and you are activating Delivering then do Knock
Do Knock if and only if you are activating Carrying_Letter_KressGazit and you are sensing Door_Closed and you are activating Delivering and you are in Office_KressGazit
#If you are activating Delivering and you are in Office_KressGazit and you are sensing Person then do Deliver_Letter
Do Deliver_Letter if and only if you are activating Delivering and you are in Office_KressGazit and you are sensing Person

