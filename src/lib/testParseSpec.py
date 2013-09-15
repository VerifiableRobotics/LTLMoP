#!/usr/bin/python

import parseSpec

regions = ['mail_room','classroom','office_campbell','office_kressgazit']
actions = ['pickup_letter','deliver_letter','knock','return_letter']
sensors = ['letter_kressgazit','letter_campbell','letter_classroom','door_closed','person']
auxProps = ['carrying_letter_kressgazit','carrying_letter_campbell','carrying_letter_classroom','delivering','locked_office','empty_office']

specText = """
# Init
Robot starts in Mail_Room with false
Environment starts with false

# Groups
Group office is Office_KressGazit, Office_Campbell, Classroom
Group letter is Letter_KressGazit, Letter_Campbell, Letter_Classroom
Group Carry_letter is Carrying_Letter_KressGazit, Carrying_Letter_Campbell, Carrying_Letter_Classroom
group future_rooms is empty

visit each future_rooms

# Correspondance mapping
#Carrying_Letter_Campbell, Carrying_Letter_KressGazit, Carrying_Letter_Classroom correspond to Letter_Campbell, Letter_KressGazit, Letter_Classroom
#Carrying_Letter_Campbell, Carrying_Letter_KressGazit, Carrying_Letter_Classroom correspond to Office_Campbell, Office_KressGazit, Classroom
Carry_letter corresponds to letter
Carry_letter corresponds to office

If you are activating any carry_letter and sensing the corresponding letter then go to the corresponding office

add to group office

# Set/Reset the indicator for whether or not the robot is carrying a letter
Each carry_letter is set on Pickup_Letter and the corresponding letter and reset on deliver_letter or return_letter

# Set/Reset the mode for whether the robot is delivering a letter, or returning it to the mail room
Do Locked_Office if and only if you activated Knock and you are sensing Door_Closed
Do Empty_Office if and only if you are not sensing Person and you are not sensing Door_Closed and you are in any office
Delivering is set on Pickup_Letter and reset on Deliver_Letter or Locked_Office or Empty_Office

# Go to the mail room to pick up any letters, or to drop off extras
#If you are not activating Carrying_Letter_KressGazit then go to Mail_Room 
If you are not activating Delivering then go to Mail_Room
Do Return_Letter if and only if you are not activating Delivering and you are in Mail_Room and you are activating any carry_letter

# When in the mail room, pick up any letters
Do Pickup_Letter if and only if you are in Mail_Room and you are sensing any letter

# Deliver any letters to their appropriate offices, if someone is there to take the letter
If you are activating any carry_letter and you are activating delivering then go to the corresponding office
Do Knock if and only if you are activating any carry_letter and you are activating Delivering and you are in any office and you are sensing Door_Closed
Do Deliver_Letter if and only if you are activating Delivering and you are in any office and you are sensing Person and you are sensing not Door_Closed

"""

sent = specText.split('\n')

#[spec,linemap,failed,LTL2LineNo] = parseSpec.writeSpec('\n'.join(sent[0:10])+'\n'+'\n'.join(sent[17:18]), sensors, regions, actions+auxProps)
[spec,linemap,failed,LTL2LineNo,internal_props] = parseSpec.writeSpec(specText, sensors, regions, actions+auxProps)

for formula in spec.values():
    print formula

