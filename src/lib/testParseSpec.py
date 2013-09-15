#!/usr/bin/python

import parseSpec

regions = ['mail_room','classroom','office_campbell','office_kressgazit']
actions = ['radio']
sensors = ['door_closed','person']
auxProps = ['carrying_letter_kressgazit','carrying_letter_campbell','carrying_letter_classroom','delivering','locked_office','empty_office']

specText = """
Group faces is empty
Group greetings is empty

faces correspond to greetings

do the corresponding greeting if and only if you are sensing each face
"""

sent = specText.split('\n')

#[spec,linemap,failed,LTL2LineNo] = parseSpec.writeSpec('\n'.join(sent[0:10])+'\n'+'\n'.join(sent[17:18]), sensors, regions, actions+auxProps)
[spec,linemap,failed,LTL2LineNo,internal_props] = parseSpec.writeSpec(specText, sensors, regions, actions+auxProps)

for formula in spec.values():
    print formula

