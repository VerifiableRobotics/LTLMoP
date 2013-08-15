# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
Pickup_Letter, 1
Deliver_Letter, 1
Knock, 1

CompileOptions:
convexify: True
fastslow: False

Customs: # List of custom propositions
Carrying_Letter_KressGazit
Carrying_Letter_Campbell
Carrying_Letter_Classroom
Delivering

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
Mail_Room = p3
Classroom = p2
others = p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16
Office_Campbell = p4
Office_KressGazit = p5

Spec: # Specification in structured English
# Init

# Groups
Group Office is Office_KressGazit and Office_Campbell and Classroom
Group Letter is Letter_KressGazit and Letter_Campbell and Letter_Classroom
Group Carry_Letter is Carrying_Letter_KressGazit and Carrying_Letter_Campbell and Carrying_Letter_Classroom

# Correspondance mapping
Office_KressGazit and Office_Campbell and Classroom correspond to Letter_KressGazit and Letter_Campbell and Letter_Classroom correspond to Carrying_Letter_KressGazit and Carrying_Letter_Campbell and Carrying_Letter_Classroom, respectively.

# Go to the mail room to pick up any letters, or to drop off extras
If you are not activating any Carry_Letter then go to Mail_Room
If you are not activating Delivering and you are activating any Carry_Letter then go to Mail_Room and do Deliver_Letter

# When in the mail room, pick up any letters, and set the corresponding Carrying propositions
If you are in Mail_Room and you are sensing any Letter then do Pickup_Letter
Each Carry_Letter is set on Pickup_Letter and the corresponding Letter and reset on Deliver_Letter and the corresponding Office

# Set/Reset the mode for whether the robot is delivering a letter, or returning it to the mail room
Delivering is set on Pickup_Letter and reset on Deliver_Letter or previously Knock and Door_Closed or any Office and not Person

# Deliver any letters to their appropriate offices, if someone is there to take the letter
If you are activating any Carry_Letter and you are activating Delivering then go to the corresponding Office
If you are in any Office and you are sensing Door_Closed and you are activating Delivering then do Knock
If you are activating Delivering and you are in any Office and you are sensing Person then do Deliver_Letter

