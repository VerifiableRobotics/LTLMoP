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
basic

Customs: # List of custom propositions
Carrying_Letter_KressGazit
Carrying_Letter_Campbell
Carrying_Letter_Classroom
Delivering
Locked_Office
Empty_Office

RegionFile: # Relative path of region description file
PR3.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
Letter_KressGazit, 1
Letter_Campbell, 1
Letter_Classroom, 1
Door_Closed, 1
Person, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Classroom = p7
Mail_Room = p6
Office_KressGazit = p4
others = p1
Office_Campbell = p5

Spec: # Specification in structured English
# Init
Robot starts in Mail_Room with false
Environment starts with false

# Groups
Group Office is Office_KressGazit, Office_Campbell, Classroom
Group Letter is Letter_KressGazit, Letter_Campbell, Letter_Classroom
Group Carry_Letter is Carrying_Letter_KressGazit, Carrying_Letter_Campbell, Carrying_Letter_Classroom

# Correspondance mapping
Carrying_Letter_Campbell, Carrying_Letter_KressGazit, Carrying_Letter_Classroom correspond to Letter_Campbell, Letter_KressGazit, Letter_Classroom
Carrying_Letter_Campbell, Carrying_Letter_kressGazit, Carrying_Letter_Classroom correspond to Office_Campbell, Office_KressGazit, Classroom

# Set/Reset the indicator for whether or not the robot is carrying a Letter
Each Carry_Letter is set on Pickup_Letter and the corresponding Letter and reset on Deliver_Letter or Return_Letter

# Set/Reset the mode for whether the robot is Delivering a Letter, or returning it to the mail room
Do Locked_Office if and only if you activated Knock and are sensing Door_Closed
Do Empty_Office if and only if you are not sensing Person and not sensing Door_Closed and in any Office
Delivering is set on Pickup_Letter and reset on Deliver_Letter or Locked_Office or Empty_Office

# Go to the mail room to pick up any Letters, or to drop off extras
If you are not activating any Carry_Letter then go to Mail_Room
If you are not activating Delivering then go to Mail_Room
Do Return_Letter if and only if you are not activating Delivering and you are in Mail_Room and activating any Carry_Letter

# When in the mail room, pick up any Letters
Do Pickup_Letter if and only if you are in Mail_Room and sensing any Letter

# Deliver any Letters to their appropriate Offices, if someone is there to take the Letter
If you are activating any Carry_Letter and activating Delivering then go to the corresponding Office
Do Knock if and only if you are activating any Carry_Letter and Delivering and you are in any Office and sensing Door_Closed
Do Deliver_Letter if and only if you were activating any Carry_Letter and were in any Office and were sensing Person

