""" LTLMoP Toolkit - createTLVinput 
    
    Module that create the input files for the TLV based synthesis algorithm.
    Its functions create the skelaton .smv file and the .pf file which
    includes the topological relation and the given spec.

    This is completely free software; please feel free to adapt or use this in
    any way you like.

    Written by Hadas Kress-Gazit (hadaskg@seas.upenn.edu)

    Last updated September 2007
"""
import numpy
import parseEnglishToLTL

def createSMVfile(fileName, numRegions, sensorList, robotPropList):
    ''' This function writes the skeleton SMV file.
    It takes as input a filename, the number of regions, the list of the
    sensor propositions and the list of robot propositions (without the regions).
    '''

    fileName = fileName + '.smv'
    smvFile = open(fileName, 'w')

    # Write the header
    smvFile.write(' -- Skeleton SMV file: ')
    smvFile.write(fileName)
    smvFile.write('\n\n\n')
    smvFile.write('MODULE main \n \t')
    smvFile.write('VAR \n')

    # Define sensor propositions
    for sensor in sensorList:
        smvFile.write('\t \t ')
        smvFile.write(sensor)
        smvFile.write(' :  boolean;\n')

    # Define the number of bits needed to encode the regions
    numBits = int(numpy.ceil(numpy.log2(numRegions)))
    for bitNum in range(numBits):
        smvFile.write('\t \t bit')
        smvFile.write(str(bitNum))
        smvFile.write(' :  boolean;\n')
        

    # Define robot propositions
    for robotProp in robotPropList:
        smvFile.write('\t \t ')
        smvFile.write(robotProp)
        smvFile.write(' :  boolean;\n')


    # close the file
    smvFile.close()
    

def createPFfile(fileName, sensorList, robotPropList, adjData, spec):
    ''' This function writes the PF file. It encodes the specification and 
    topological relation. 
    It takes as input a filename, the list of the
    sensor propositions, the list of robot propositions (without the regions),
    the adjacency data (transition data structure) and
    a dictionary containing the specification strings.
    '''

    fileName = fileName + '.pf'
    pfFile = open(fileName, 'w')

    numBits = int(numpy.ceil(numpy.log2(len(adjData))))
    bitEncode = parseEnglishToLTL.bitEncoding(len(adjData), numBits)
    currBitEnc = bitEncode['current']
    nextBitEnc = bitEncode['next']
    
    # Write the header and begining of the formula
    pfFile.write(' -- PF file : ')
    pfFile.write(fileName)
    pfFile.write('\n\n')
    pfFile.write(' Load "new_synthesis.tlv";\n\n')
    pfFile.write(' Let ts := compile_spec(\n')
    pfFile.write(' \t ltl( \n\t\t(\n')

    # Write the environment assumptions
    # from the 'spec' input 
    pfFile.write(spec['EnvInit'])
    pfFile.write(spec['EnvTrans'])
    pfFile.write(spec['EnvGoals'])
    pfFile.write(' \t\t)\n\t\t->\n\t\t(')

    # Write the desired robot behavior
    pfFile.write(spec['SysInit'])

    # The topological relation (adjacency)
    for Origin in range(len(adjData)):
        # from region i we can stay in region i
        pfFile.write('\t\t\t []( (')
        pfFile.write(currBitEnc[Origin])
        pfFile.write(') -> ( (')
        pfFile.write(nextBitEnc[Origin])
        pfFile.write(')')
        
        for dest in range(len(adjData)):
            if adjData[Origin][dest]:
                # not empty, hence there is a transition
                pfFile.write('\n\t\t\t\t\t\t\t\t\t| (')
                pfFile.write(nextBitEnc[dest])
                pfFile.write(') ')

        # closing this region
        pfFile.write(' ) ) & \n ')
    

    # The rest of the spec
    pfFile.write(spec['SysTrans'])
    pfFile.write(spec['SysGoals'])
    # Close the LTL formula
    pfFile.write(' \t\t)\n\t),\n ')

    # Write the list of propositions
    pfFile.write(' \t')
    # Write sensor propositions
    for sensor in sensorList:
        pfFile.write(sensor)
        if not sensor == sensorList[-1]:
            # sensors have to be distinct so we can check for
            # the last one this way
            pfFile.write(' & ')
    pfFile.write(' , ')
    # Write the bits needed to encode the regions
    numBits = int(numpy.ceil(numpy.log2(len(adjData))))
    for bitNum in range(numBits):
        pfFile.write('bit')
        pfFile.write(str(bitNum))
        if not bitNum == numBits-1:
            pfFile.write(' & ')
            
    # Define robot propositions
    for robotProp in robotPropList:
        pfFile.write(' & ')
        pfFile.write(robotProp)
        # Better to put the '&' before and remove the last bit numbers '&' in cases there are no actions.
##        if not robotProp == robotPropList[-1]:
##            # propositions have to be distinct so we can check for
##            # the last one this way
##            pfFile.write(' & ')
    



    # Write the rest
    pfFile.write(' );\n\n')
    pfFile.write(' Settime;\n')
    pfFile.write(' check_realizability ts, (ts + 1);\n')
    pfFile.write(' find_strategy some_aut_name, ts, (ts + 1);\n')
    pfFile.write(' optimize_aut some_aut_name;\n')
    pfFile.write(' optimize_aut some_aut_name;\n')
    pfFile.write(' optimize_aut some_aut_name;\n')
    pfFile.write(' Chktime;\n')
    pfFile.write(' log "')
    pfFile.write(fileName.replace('.pf','.aut'))
    pfFile.write('";\n')
    pfFile.write(' print_aut some_aut_name;\n')
    pfFile.write(' log;\n')

    # close the file
    pfFile.close()


