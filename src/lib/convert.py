import numpy, re
import parseEnglishToLTL
import textwrap

def createAnzuFile(fileName, sensorList, robotPropList, adjData, spec):
    ''' This function writes the Anzu file. It encodes the specification and 
    topological relation. 
    It takes as input a filename, the list of the
    sensor propositions, the list of robot propositions (without the regions),
    the adjacency data (transition data structure) and
    a dictionary containing the specification strings.
    '''

    fileName = fileName + '.anzu'
    anzuFile = open(fileName, 'w')

    numBits = int(numpy.ceil(numpy.log2(len(adjData))))
    bitEncode = parseEnglishToLTL.bitEncoding(len(adjData), numBits)
    currBitEnc = bitEncode['current']
    nextBitEnc = bitEncode['next']
    
    # Write the header and begining of the formula
    anzuFile.write('[INPUT_VARIABLES]\n')
    anzuFile.writelines( "%s;\n" % item for item in sensorList )
    anzuFile.write('\n\n')


    anzuFile.write('[OUTPUT_VARIABLES]\n')
    anzuFile.writelines( "%s;\n" % item for item in robotPropList)
    anzuFile.write('\n\n')



    # Write the environment assumptions
    # from the 'spec' input 
    anzuFile.write('[ENV_INITIAL]\n')
    anzuFile.write(reformat(spec['EnvInit'], sensorList, robotPropList, numBits));
    anzuFile.write('\n\n')


    anzuFile.write('[ENV_TRANSITIONS]\n')
    anzuFile.write(reformat(spec['EnvTrans'], sensorList, robotPropList, numBits));
    anzuFile.write('\n\n')


    anzuFile.write('[ENV_FAIRNESS]\n')
    anzuFile.write(reformat(spec['EnvGoals']+')', sensorList, robotPropList, numBits))
    anzuFile.write(';\n\n')

    

    # Write the desired robot behavior
    anzuFile.write('[SYS_INITIAL]\n')
    anzuFile.write(reformat(spec['SysInit'], sensorList, robotPropList, numBits));
    anzuFile.write('\n\n')


    anzuFile.write('[SYS_TRANSITIONS]\n')
    # The topological relation (adjacency)
    for Origin in range(len(adjData)):
        # from region i we can stay in region i
        anzuFile.write('\t\t\t G( (')
        anzuFile.write(reformat(currBitEnc[Origin], sensorList, robotPropList, numBits))
        anzuFile.write(') -> ( (')
        anzuFile.write(reformat(nextBitEnc[Origin], sensorList, robotPropList, numBits))
        anzuFile.write(')')
        
        for dest in range(len(adjData)):
            if adjData[Origin][dest]:
                # not empty, hence there is a transition
                anzuFile.write('\n\t\t\t\t\t\t\t\t\t+ (')
                anzuFile.write(reformat(nextBitEnc[dest], sensorList, robotPropList, numBits))
                anzuFile.write(') ')

    # The rest of the system transitions
    anzuFile.write(reformat(spec['SysTrans'], sensorList, robotPropList, numBits));
    anzuFile.write('\n\n')


    anzuFile.write('[SYS_FAIRNESS]\n')
    anzuFile.write(reformat(spec['SysGoals'].strip()+')', sensorList, robotPropList, numBits))
    anzuFile.write(';\n\n')

    # close the file
    anzuFile.close()

def reformat(str, sensorList, robotPropList, numBits):
    clean = str.replace('[]', 'G').replace('<>', 'F').replace('& \n',';\n').replace('|', '+').replace('&', '*').replace('next', 'X').replace('GF', 'G(F');
    for i in sensorList:
        clean = clean.replace(i, i+'=1');
    for i in range(0,numBits):
        clean = clean.replace('s.bit'+`i`, 's.bit'+`i`+'=1');
    for i in robotPropList:
        clean = clean.replace(i, i+'=1');
    nextRE = re.compile('!([^\s]*)=1', re.VERBOSE);
    clean = nextRE.sub(r'\1=0', clean);
    
    #otherRE = re.compile('X\(([^=\s]*)\)', re.VERBOSE);
    #clean = otherRE.sub(r'X(\1=1)', clean);
    return clean;
