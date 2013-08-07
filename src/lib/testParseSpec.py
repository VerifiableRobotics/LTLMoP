#!/usr/bin/python

import parseSpec

regions = ['mail_room','classroom','office_campbell','office_kressgazit']
actions = ['pickup_letter','deliver_letter','knock']
sensors = ['letter_kressgazit','letter_campbell','letter_classroom','door_closed','person']
auxProps = ['carrying_letter_kressgazit','carrying_letter_campbell','carrying_letter_classroom','delivering']

spec = """# Init

# Groups
Group Office is Office_KressGazit, Office_Campbell, Classroom
Group Letter is Letter_KressGazit, Letter_Campbell, Letter_Classroom
Group Carry_Letter is Carrying_Letter_KressGazit, Carrying_Letter_Campbell, Carrying_Letter_Classroom

# Correspondance mapping
Carrying_Letter_Campbell, Carrying_Letter_KressGazit, Carrying_Letter_Classroom correspond to Letter_Campbell, Letter_KressGazit, Letter_Classroom
Carrying_Letter_Campbell, Carrying_Letter_KressGazit, Carrying_Letter_Classroom correspond to Office_Campbell, Office_KressGazit, Classroom

# Go to the mail room to pick up any letters, or to drop off extras
If you are not activating any Carry_Letter then go to Mail_Room
If you are not activating Delivering and you are activating any Carry_Letter then go to Mail_Room and Deliver_Letter

# When in the mail room, pick up any letters, and set the corresponding Carrying propositions
If you are in Mail_Room and you are sensing any Letter then do Pickup_Letter
Each Carry_Letter is set on Pickup_Letter and the corresponding Letter and reset on Deliver_Letter and the corresponding Office

# Set/Reset the mode for whether the robot is delivering a letter, or returning it to the mail room
Delivering is set on Pickup_Letter and reset on Deliver_Letter or Knock and Door_Closed or any Office and not Person

# Deliver any letters to their appropriate offices, if someone is there to take the letter
If you are activating any Carry_Letter and you are activating Delivering then go to the corresponding Office
If you are in any Office and you are sensing Door_Closed and you are activating Delivering then do Knock
If you are activating Delivering and you are in any Office and you are sensing Person then do Deliver_Letter
"""

sent = spec.split('\n')

[spec,linemap,failed,LTL2LineNo] = parseSpec.writeSpec('\n'.join(sent[0:10])+'\n'+'\n'.join(sent[17:18]), sensors, regions, actions+auxProps)

for formula in spec.values():
    print formula

