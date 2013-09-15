#!/usr/bin/python

import parseSpec

regions = ['hallN', 'closet','mailRoom','hallC','lounge','hallW']
actions = ['resynthesize']
sensors = ['letter1', 'letter2', 'newletter']
auxProps = []

specText = """
# Letters that we can detect
Group Letters is empty

# Regions that each letter needs to end up in.
Group Offices is empty

# Regions to patrol if you are not carrying letters
Group PatrolRooms is mailRoom, hallW, hallN

# C(letters) = {Destinations}
Letters correspond to Offices

#robot starts in mailRoom with false
#environment starts with false

#### Spec-rewriting and resynthesis mechanics ####
If you are in any offices then do not resynthesize


# This is only triggered if we see a letter we haven't ever seen before
If you are sensing newLetter then add to group Letters and resynthesize

#### Letter delivery specification ####
If you are sensing any Letters then go to the corresponding Office

# No spurious pickups
#Do pick_up if and only if you are sensing any Letters

# Go back to the mailroom if we have nothing else to do
If you are not sensing any Letters then visit each PatrolRoom

# Environment assumption: Be nice don't show us letters all the time
Infinitely often not newLetter

# F/S stuffs
If you are sensing newLetter then stay there
"""

sent = specText.split('\n')

#[spec,linemap,failed,LTL2LineNo] = parseSpec.writeSpec('\n'.join(sent[0:10])+'\n'+'\n'.join(sent[17:18]), sensors, regions, actions+auxProps)
[spec,linemap,failed,LTL2LineNo,internal_props] = parseSpec.writeSpec(specText, sensors, regions, actions+auxProps)

for formula in spec.values():
    print formula

