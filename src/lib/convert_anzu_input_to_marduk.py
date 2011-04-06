#!/usr/bin/env python

##  ===========================================================================
##  Author: Georg Hofferek <georg.hofferek@iaik.tugraz.at>
##
##  Copyright (c) 2009, 2010 by Graz University of Technology 
##
##  This is free software; you can redistribute it and/or
##  modify it under the terms of the GNU Lesser General Public
##  License as published by the Free Software Foundation; either
##  version 2 of the License, or (at your option) any later version.
##
##  This software is distributed in the hope that it will be useful,
##  but WITHOUT ANY WARRANTY; without even the implied warranty of
##  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
##  Lesser General Public License for more details.
##
##  You should have received a copy of the GNU Lesser General Public
##  License along with this library; if not, write to the Free Software
##  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA.
##
##  For more information about this software see <http://rat.fbk.eu/ratsy>
##  or email to the RATSY Team <ratsy@list.fbk.eu>.
##  Please report bugs to <ratsy@list.fbk.eu>.
##
##  ===========================================================================




# This file contains a utility program to convert Anzu input files
# to Marduk input files, or RAT project files.
# call it from the commandline with the following options:
#
# -i <Anzu_Input_file>  -o <File_Name_For_Marduk_Input_File>
# Use option -r or --rat to produce RATSY project files instead of
# Marduk input files.



from optparse import OptionParser
import xml.dom.minidom as dom
import re

parser = OptionParser()

parser.add_option("-i", "--input", dest="input_file",
                  help="Input File, Specification in Anzu format")
parser.add_option("-o", "--output", dest="output_file",
                  help="Output File in Marduk/RAST format")
parser.add_option("-v", "--verbose", action="store_true", dest="verbose",
                  default=False, help="Enable verbose output")
parser.add_option("-r", "--rat", dest="rat", action="store_true", default=False,
                  help="Create a RAT project file, instead of a Marduk input file")

(options, args) = parser.parse_args();

if options.input_file == None:
    print "WARNING:! No input file specified! Using 'spec.cfg' as default."
    input_file = "spec.cfg"
else:
    input_file = options.input_file


# Read input file
input = open(input_file, 'r')
input_lines = input.readlines()
input.close()

mode = None
incomplete = ''
sys_vars = []
env_vars = []
sys_init = []
env_init = []
sys_tran = []
env_tran = []
sys_fair = []
env_fair = []

warned_about_ordering = False

for line in input_lines:
    line = re.sub('^\s*','',line) # Strip leading whitespace

    if len(line) == 0:
        continue                   # ignore emtpy lines
        
    if line[0] == '#':
        continue                  # ignore comment lines

    line = re.sub('#.*$', '', line)  # remove trailing comments

    if line[0] == '[':
        # Switch to new mode
        line = re.sub('\[|\]|\s*','',line)  # remove the square brackets and any further whitespace
        mode = line
        continue

    line = re.sub('\s*$','',line)        # Strip trailing whitespace
    
    if line[len(line)-1] != ';':
        incomplete = incomplete + line   # join incomplete lines
        continue

    if line[len(line)-1] == ';':
        line = incomplete + line   # line is complete
        incomplete = ''
        line = line[0:len(line)-1] # remove trailing semicolon

    line = re.sub('\s','',line)    # remove all remaining whitespace
    line = re.sub('\*','&&',line)   # replacing * with &&
    line = re.sub('\+','||',line)   # replacing + with ||

    if options.verbose:
        print "-------------------------------"
        print "Turning "
        print line
    line = re.sub('(X\([^\)]*\))', r'(\1)', line)   # Parenthesize X statements to cope with operator-precedence differences,
                                                        # i.e. turn X(h0=0)*X(h1=0) into (X(h0=0))*(X(h1=1))
                                                        # Not entirely sure whether this is correct in ALL cases.
    if options.verbose:
        print "into"
        print line

    
        print 'Processing:', mode
        
    if mode == 'INPUT_VARIABLES':
        env_vars.append(line)
    elif mode == 'OUTPUT_VARIABLES':
        sys_vars.append(line)
    elif mode == 'ENV_INITIAL':
        env_init.append(line)
    elif mode == 'SYS_INITIAL':
        sys_init.append(line)
    elif mode == 'ENV_TRANSITIONS':
        env_tran.append(line)
    elif mode == 'SYS_TRANSITIONS':
        sys_tran.append(line)
    elif mode == 'ENV_FAIRNESS':
        env_fair.append(line)
    elif mode == 'SYS_FAIRNESS':
        sys_fair.append(line)
    elif mode == 'FORCE_ORDERING':
        if not warned_about_ordering:
            print "WARNING! Ignoring fixed variable ordering!"
            warned_about_ordering = True
    else:
        print "ERROR! Input file invalid! Found unexpected token '", mode,"'!"

pass # for line in input_lines

print '-------------------------'
print 'Found:'
print 'env_vars:', len(env_vars)
print 'sys_vars:', len(sys_vars)
print 'env_init:', len(env_init)
print 'sys_init:', len(sys_init)
print 'env_tran:', len(env_tran)
print 'sys_tran:', len(sys_tran)
print 'env_fair:', len(env_fair)
print 'sys_fair:', len(sys_fair)

signals = dom.Element('signals')

for var in env_vars:
    signal = dom.Element('signal')
    name = dom.Element('name')
    kind = dom.Element('kind')
    type = dom.Element('type')
    notes = dom.Element('notes')

    text = dom.Text()
    text.data = var
    name.appendChild(text)

    text = dom.Text()
    text.data = 'E'
    kind.appendChild(text)

    text = dom.Text()
    text.data = 'boolean'
    type.appendChild(text)

    text = dom.Text()
    text.data = ' '
    notes.appendChild(text)
    
    signal.appendChild(name)
    signal.appendChild(kind)
    signal.appendChild(type)
    signal.appendChild(notes)
    
    signals.appendChild(signal)
pass   # for var in env_vars

for var in sys_vars:
    signal = dom.Element('signal')
    name = dom.Element('name')
    kind = dom.Element('kind')
    type = dom.Element('type')
    notes = dom.Element('notes')

    text = dom.Text()
    text.data = var
    name.appendChild(text)

    text = dom.Text()
    text.data = 'S'
    kind.appendChild(text)

    text = dom.Text()
    text.data = 'boolean'
    type.appendChild(text)

    text = dom.Text()
    text.data = ' '
    notes.appendChild(text)
    
    signal.appendChild(name)
    signal.appendChild(kind)
    signal.appendChild(type)
    signal.appendChild(notes)
    
    signals.appendChild(signal)
pass   # for var in sys_vars
   
##########################################

requirements = dom.Element('requirements')

count = 0
for init in env_init:
    requirement = dom.Element('requirement')
    name = dom.Element('name')
    property = dom.Element('property')
    kind = dom.Element('kind')
    notes = dom.Element('notes')
    toggled = dom.Element('toggled')

    text = dom.Text()
    text.data = 'env init ' + str(count)
    name.appendChild(text)

    text = dom.Text()
    text.data = init
    property.appendChild(text)
    
    text = dom.Text()
    text.data = 'A'
    kind.appendChild(text)

    text = dom.Text()
    text.data = ' '
    notes.appendChild(text)

    text = dom.Text()
    text.data = '1'
    toggled.appendChild(text)
    
    requirement.appendChild(name)
    requirement.appendChild(property)
    requirement.appendChild(kind)
    requirement.appendChild(notes)
    requirement.appendChild(toggled)
    
    requirements.appendChild(requirement)
    count = count + 1
pass   # for var in env_init

count = 0
for init in sys_init:
    requirement = dom.Element('requirement')
    name = dom.Element('name')
    property = dom.Element('property')
    kind = dom.Element('kind')
    notes = dom.Element('notes')
    toggled = dom.Element('toggled')

    text = dom.Text()
    text.data = 'sys init ' + str(count)
    name.appendChild(text)

    text = dom.Text()
    text.data = init
    property.appendChild(text)
    
    text = dom.Text()
    text.data = 'G'
    kind.appendChild(text)

    text = dom.Text()
    text.data = ' '
    notes.appendChild(text)

    text = dom.Text()
    text.data = '1'
    toggled.appendChild(text)
    
    requirement.appendChild(name)
    requirement.appendChild(property)
    requirement.appendChild(kind)
    requirement.appendChild(notes)
    requirement.appendChild(toggled)
    
    requirements.appendChild(requirement)
    count = count + 1
pass   # for var in sys_init

count = 0
for init in env_tran:
    requirement = dom.Element('requirement')
    name = dom.Element('name')
    property = dom.Element('property')
    kind = dom.Element('kind')
    notes = dom.Element('notes')
    toggled = dom.Element('toggled')

    text = dom.Text()
    text.data = 'env tran ' + str(count)
    name.appendChild(text)

    text = dom.Text()
    text.data = init
    property.appendChild(text)
    
    text = dom.Text()
    text.data = 'A'
    kind.appendChild(text)

    text = dom.Text()
    text.data = ' '
    notes.appendChild(text)

    text = dom.Text()
    text.data = '1'
    toggled.appendChild(text)
    
    requirement.appendChild(name)
    requirement.appendChild(property)
    requirement.appendChild(kind)
    requirement.appendChild(notes)
    requirement.appendChild(toggled)
    
    requirements.appendChild(requirement)
    count = count + 1
pass   # for var in env_tran

count = 0
for init in sys_tran:
    requirement = dom.Element('requirement')
    name = dom.Element('name')
    property = dom.Element('property')
    kind = dom.Element('kind')
    notes = dom.Element('notes')
    toggled = dom.Element('toggled')

    text = dom.Text()
    text.data = 'sys tran ' + str(count)
    name.appendChild(text)

    text = dom.Text()
    text.data = init
    property.appendChild(text)
    
    text = dom.Text()
    text.data = 'G'
    kind.appendChild(text)

    text = dom.Text()
    text.data = ' '
    notes.appendChild(text)

    text = dom.Text()
    text.data = '1'
    toggled.appendChild(text)
    
    requirement.appendChild(name)
    requirement.appendChild(property)
    requirement.appendChild(kind)
    requirement.appendChild(notes)
    requirement.appendChild(toggled)
    
    requirements.appendChild(requirement)
    count = count + 1
pass   # for var in sys_tran

count = 0
for init in env_fair:
    requirement = dom.Element('requirement')
    name = dom.Element('name')
    property = dom.Element('property')
    kind = dom.Element('kind')
    notes = dom.Element('notes')
    toggled = dom.Element('toggled')

    text = dom.Text()
    text.data = 'env fair ' + str(count)
    name.appendChild(text)

    text = dom.Text()
    text.data = init
    property.appendChild(text)
    
    text = dom.Text()
    text.data = 'A'
    kind.appendChild(text)

    text = dom.Text()
    text.data = ' '
    notes.appendChild(text)

    text = dom.Text()
    text.data = '1'
    toggled.appendChild(text)
    
    requirement.appendChild(name)
    requirement.appendChild(property)
    requirement.appendChild(kind)
    requirement.appendChild(notes)
    requirement.appendChild(toggled)
    
    requirements.appendChild(requirement)
    count = count + 1
pass   # for var in env_fair

count = 0
for init in sys_fair:
    requirement = dom.Element('requirement')
    name = dom.Element('name')
    property = dom.Element('property')
    kind = dom.Element('kind')
    notes = dom.Element('notes')
    toggled = dom.Element('toggled')

    text = dom.Text()
    text.data = 'sys fair ' + str(count)
    name.appendChild(text)

    text = dom.Text()
    text.data = init
    property.appendChild(text)
    
    text = dom.Text()
    text.data = 'G'
    kind.appendChild(text)

    text = dom.Text()
    text.data = ' '
    notes.appendChild(text)

    text = dom.Text()
    text.data = '1'
    toggled.appendChild(text)
    
    requirement.appendChild(name)
    requirement.appendChild(property)
    requirement.appendChild(kind)
    requirement.appendChild(notes)
    requirement.appendChild(toggled)
    
    requirements.appendChild(requirement)
    count = count + 1
pass   # for var in sys_fair



project = dom.Element('project')
project.appendChild(signals)
project.appendChild(requirements)

doc = dom.Document()
doc.appendChild(project)
    

if options.output_file == None:
    print "WARNING:! No output file specified! Using 'input.xml' as default."
    output_file = "input.xml"
else:
    output_file = options.output_file

output = open(output_file, 'w')
doc.writexml(output, '', '  ', '\n')
output.close()

if options.rat:
    file = open(output_file, 'r')
    lines = file.readlines()
    file.close()
    content = "".join(lines)
    content = re.sub("</project>", """
  <property_assurance>
    <possibilities/>
    <assertions/>
  </property_assurance>
  <property_simulation/>
  <categories>
    <category>
      <name>
        New
      </name>
      <editable>
        no
      </editable>
      <notes>
        This is the category of those traces that have been just created
      </notes>
    </category>
    <category>
      <name>
        Default
      </name>
      <editable>
        no
      </editable>
      <notes>
        This is the default category for traces
      </notes>
    </category>
    <category>
      <name>
        Out of Date
      </name>
      <editable>
        no
      </editable>
      <notes>
        Contains the traces whose dependencies might be no longer consistent
      </notes>
    </category>
    <category>
      <name>
        Trash
      </name>
      <editable>
        no
      </editable>
      <notes>
        Contains the traces that have been deleted
      </notes>
    </category>
  </categories>
  <active_view>
    re
  </active_view>
  <notes>
    
  </notes>
</project>""", content)

    file = open(options.output_file, 'w')
    file.write(content)
    file.close()

print "Successfully converted", input_file, "to", output_file
        
