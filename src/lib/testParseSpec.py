#!/usr/bin/python

import parseSpec
import cProfile

regions = ['room1','room2','room3']
actions = ['resynthesize']
sensors = ['letter1', 'letter2', 'newletter']
auxProps = []

specText = """
infinitely often not room1 or room2 and room3
"""

sent = specText.split('\n')

#[spec,linemap,failed,LTL2LineNo] = parseSpec.writeSpec('\n'.join(sent[0:10])+'\n'+'\n'.join(sent[17:18]), sensors, regions, actions+auxProps)
[spec,linemap,failed,LTL2LineNo,internal_props] = parseSpec.writeSpec(specText, sensors, regions, actions+auxProps)
# cProfile.run('[spec,linemap,failed,LTL2LineNo,internal_props] = parseSpec.writeSpec(specText, sensors, regions, actions+auxProps)')
#print('\nLTL Formulas: \n')

#for formula in spec.values():
#    print formula

