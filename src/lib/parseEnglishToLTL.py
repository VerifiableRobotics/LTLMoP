""" 
    ===========================================================
    parseEnglishToLTL.py - Structured English to LTL Translator 
    ===========================================================
    
    Module that parses a set of structured English sentences into the
    corresponding LTL subformulas.
"""
import re
import numpy

def writeSpec(text, sensorList, regionList, robotPropList):
    ''' This function creates the Spec dictionary that contains the parsed LTL
        subformulas. It takes the text that contains the structured English,
        the list of sensor propositions, the list containing
        the region names and the list of robot propositions (other than regions). 
    '''

    # Prepend "e." or "s." to propositions for JTLV
    for i, sensor in enumerate(sensorList):
        text = re.sub("\\b"+sensor+"\\b", "e." + sensor, text)
        sensorList[i] = "e." + sensorList[i]

    for i, prop in enumerate(robotPropList):
        text = re.sub("\\b"+prop+"\\b", "s." + prop, text)
        robotPropList[i] = "s." + robotPropList[i]

    # initializing the dictionary
    spec = {}
    spec['EnvInit']= ''
    spec['EnvTrans']= ''
    spec['EnvGoals']= ''
    spec['SysTrans']= ''
    spec['SysGoals']= ''

    # List of all robot prpositions
    allRobotProp = regionList + robotPropList

    # Define the number of bits needed to encode the regions
    numBits = int(numpy.ceil(numpy.log2(len(regionList))))

    # creating the region bit encoding
    bitEncode = bitEncoding(len(regionList),numBits)
    currBitEnc = bitEncode['current']
    nextBitEnc = bitEncode['next']

    # Setting the system initial formula to allow only valid
    #  region encoding. This may be redundent if an initial region is
    #  specified, but it is here to ensure the system cannot start from
    #  an invalid encoding
    spec['SysInit']= '\t\t\t( ' + currBitEnc[0] + ' \n'
    for regionInd in range(1,len(regionList)):
        spec['SysInit'] = spec['SysInit'] + '\t\t\t\t | ' + currBitEnc[regionInd] + '\n'
    spec['SysInit'] = spec['SysInit'] + '\t\t\t) & \n'
    

    # Regular expressions to help us out
    EnvInitRE = re.compile('^(environment|env) starts with',re.IGNORECASE)
    SysInitRE = re.compile('^(robot |you |)starts?',re.IGNORECASE)
    IfThenRE = re.compile('if (?P<cond>.+) then (?P<req>.+)',re.IGNORECASE)
    UnlessRE = re.compile('(?P<req>.+) unless (?P<cond>.+)',re.IGNORECASE)
    IffRE = re.compile('(?P<req>.+) if and only if (?P<cond>.+)',re.IGNORECASE)
    CommentRE = re.compile('^\s*#',re.IGNORECASE)
    LivenessRE = re.compile('(go to|visit|infinitely often do|infinitely often sense|infinitely often)',re.IGNORECASE)
    SafetyRE = re.compile('(always|always do |do |always sense |sense )',re.IGNORECASE)
    StayRE = re.compile('(stay there|stay)',re.IGNORECASE)
    EventRE = re.compile('(?P<prop>[\w\.]+) is set on (?P<setEvent>.+) and reset on (?P<resetEvent>.+)',re.IGNORECASE)


    # Creating the 'Stay' formula - it is a constant formula given the number of bits.
    StayFormula = createStayFormula(numBits)

    lineInd = 0

    # iterate over the lines in the file
    for line in text.split("\n"):

        lineInd = lineInd + 1

        # If it is an empty line, ignore it
        if re.search('^(\s*)\n$',line) or line == '':
            continue
            
        # If the sentence is a comment, ignore it
        if CommentRE.search(line):
            continue

        # If there is a newline at the end, remove it
        if re.search('\n$',line):
            line = re.sub('\n$','',line)
        # If there are ',' or '.', remove then
        line = line.replace('\B.\B',' ') # Leave periods that are in the middle of words.
        line = line.replace(',',' ')


        # If the sentence describes the initial state of the environemnt
        if EnvInitRE.search(line):

            # remove the first words     
            EnvInit = EnvInitRE.sub('',line)

            # parse the rest and return it to spec['EnvInit']
            LTLsubformula = parseInit(EnvInit,sensorList,lineInd)
            spec['EnvInit']= spec['EnvInit'] + LTLsubformula
            
        # If the sentence describes the initial state of the robot
        elif SysInitRE.search(line):
            # remove the first words     
            SysInit = SysInitRE.sub('',line)

            RegInit = ''
            ActInit = ''
            LTLRegSubformula = ''
            LTLActSubformula = ''
            
            # divide, if needed, into region initial condition and action
            # (other propositions) initial conditions  
            if re.search('in (.+) with (.+)',SysInit):
                RegInit = re.search('in (.+) with (.+)',SysInit).groups()[0]
                ActInit = re.search('in (.+) with (.+)',SysInit).groups()[1]

            # Only region initial condition    
            elif ' in ' in SysInit:
                RegInit = SysInit.replace(' in ', ' ')

            # Only action initial condition    
            elif ' with ' in SysInit:
                ActInit = SysInit.replace(' with ', ' ')

            if RegInit:
                # parse regions
                LTLRegSubformula = parseInit(RegInit,regionList,lineInd)
            if ActInit:
                # parse Actions
                LTLActSubformula = parseInit(ActInit,robotPropList,lineInd)
            
            spec['SysInit']= spec['SysInit'] + LTLRegSubformula + LTLActSubformula

                

        # If the sentence is a conditional 
        elif IfThenRE.search(line) or UnlessRE.search(line) or IffRE.search(line) :
            if IfThenRE.search(line):
                CondParts = IfThenRE.search(line)
                CondType = 'IfThen'
            elif UnlessRE.search(line):
                CondParts = UnlessRE.search(line)
                CondType = 'Unless'
            else :
                CondParts = IffRE.search(line)
                CondType = 'IFF'

            # Extract the 2 pieces (condition and requirement)  
            Condition = CondParts.group('cond')
            Requirement = CondParts.group('req')

            # Figure out what the requirement is and parse it
            if SafetyRE.search(Requirement):
                # remove first words
                Requirement = SafetyRE.sub(' ',Requirement)
                # and parse requirement
                ReqFormulaInfo = parseSafety(Requirement,sensorList,allRobotProp,lineInd)
            elif LivenessRE.search(Requirement):
                # remove first words
                Requirement = LivenessRE.sub(' ',Requirement)
                # and parse requirement
                ReqFormulaInfo = parseLiveness(Requirement,sensorList,allRobotProp,lineInd)
            elif StayRE.search(Requirement):
                ReqFormulaInfo = {}
                # The formula - adding the '\t\t\t []' so it will be added to the conditional
                # ( a bit of a hack, but it is just to make it a 'safety' requirement)
                ReqFormulaInfo['formula'] = '\t\t\t []' + StayFormula
                # the type - it has to be the system safety requirement
                ReqFormulaInfo['type'] = 'SysTrans'
                
            else:
                print 'ERROR(13): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because could not resolve the requirement:'
                print Requirement
                print ''
                continue

            # Parse the condition and add it to the requirement
            CondFormulaInfo = parseConditional(Condition,ReqFormulaInfo,CondType,sensorList,allRobotProp,lineInd)

            spec[CondFormulaInfo['type']] = spec[CondFormulaInfo['type']] + CondFormulaInfo['formula']


        # An event definition
        elif EventRE.search(line):
            # get the groups
            EventParts = EventRE.search(line)
            EventProp = EventParts.group('prop') 
            SetEvent = EventParts.group('setEvent') 
            ResetEvent = EventParts.group('resetEvent') 

            # Formulas defining when the proposition is true and false
            EventFormula = parseEvent(EventProp,SetEvent,ResetEvent,sensorList,allRobotProp,lineInd)

            spec['SysTrans'] = spec['SysTrans'] + EventFormula


        # A safety requirement
        elif SafetyRE.search(line):
            # remove the first words     
            SafetyReq = SafetyRE.sub('',line)

            # parse the safety requirement
            formulaInfo = parseSafety(SafetyReq,sensorList,allRobotProp,lineInd)

            spec[formulaInfo['type']] = spec[formulaInfo['type']] + formulaInfo['formula']

            
        # A 'Go to and stay there' requirement
        elif LivenessRE.search(line) and StayRE.search(line):
            # remove the liveness words    
            LivenessReq = LivenessRE.sub('',line)
            # remove the 'and stay there'      
            LivenessReq = LivenessReq.replace(' and stay there','')

            # parse the liveness requirement
            formulaInfo = parseLiveness(LivenessReq,sensorList,allRobotProp,lineInd)

            # If could not parse the liveness
            if formulaInfo['formula']=='':
                print 'ERROR(14): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because could not parse liveness requirement'
                continue
                
            # If not SysGoals, then it is an error
            if not formulaInfo['type']=='SysGoals':
                print 'ERROR(15): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because the requirement is not system liveness'
                continue

            # Add the liveness ('go to') to the spec
            spec[formulaInfo['type']] = spec[formulaInfo['type']] + formulaInfo['formula']

            # add the 'stay there' as a condition (if R then stay there)
            regCond = formulaInfo['formula'].replace('\t\t\t []<>','')
            regCond = regCond.replace('& \n','')
            condStayFormula = '\t\t\t [](' + regCond + ' -> ' + StayFormula + ') & \n'

            spec['SysTrans'] = spec['SysTrans'] + condStayFormula


        # A liveness requirement
        elif LivenessRE.search(line):
            # remove the first words     
            LivenessReq = LivenessRE.sub('',line)

            # parse the liveness requirement
            formulaInfo = parseLiveness(LivenessReq,sensorList,allRobotProp,lineInd)

            spec[formulaInfo['type']] = spec[formulaInfo['type']] + formulaInfo['formula']

        # Cannot parse
        else:
            print 'ERROR(16): Could not parse the sentence in line '+ str(lineInd)+' :'
            print line
            print 'because it is not in a known format'
            print ''
      

    # replace all region names with the bit encoding
    for key in spec:
        spec[key] = replaceRegionName(spec[key],bitEncode,regionList)
        
    
    # Setting all empty subformulas to TRUE, and removing last & in 'EnvGoals' and 'SysGoals'
    if spec['EnvInit'] == '':
        spec['EnvInit'] = '\t\t\tTRUE & \n'

    if spec['EnvTrans'] == '':
        spec['EnvTrans'] = '\t\t\t[](TRUE) & \n'
   
    if spec['EnvGoals'] == '':
        spec['EnvGoals'] = '\t\t\t[]<>(TRUE)'
    else:
        # remove last &
        spec['EnvGoals'] = re.sub('& \n$','\n',spec['EnvGoals'])

    # No need to change anything in SysTrans,
    # since the transition relation is encoded anyway
   
    if spec['SysGoals'] == '':
        spec['SysGoals'] = '\t\t\t[]<>(TRUE)'
    else:
        # remove last &
        spec['SysGoals'] = re.sub('& \n$','\n',spec['SysGoals'])



    # Check for unused sensor or robot propositions
    specstr = ''
    # put all Trans and Goal formulas into one string for ease of search
    for key in spec:
        if not 'Init' in key:
            specstr = specstr + spec[key]
    # search the string
    unusedProp = []
    for prop in sensorList + robotPropList:
        if prop in specstr:
            continue
	else:
            unusedProp = unusedProp + [prop]
    # if there are unused propositions, print out a warning
    if unusedProp:
        print '##############################################'
        print 'Warning:'
        print 'The following propositions seem to be unused:'
        print unusedProp
        print 'They should be removed from the proposition lists\n'
    

    return spec


def parseInit(sentence,PropList,lineInd):
    ''' This function creates the LTL formula representing the initial conditions.
        It takes the sentence and PropList - a list of propositions (to check that only 'legal'
        propositions are used) and 'lineInd' that indicates which line is being processed.
        Returns the LTL formula as a string.
    '''

    FalseRE = re.compile('false',re.IGNORECASE)
    TrueRE = re.compile('true',re.IGNORECASE)
    

    if FalseRE.search(sentence):
        # All propositions should be false
        tempFormula = ''
        for prop in PropList:
            tempFormula = tempFormula + '\t\t\t !' + prop + ' & \n'

        LTLsubformula = tempFormula
        return LTLsubformula

    if TrueRE.search(sentence):
        # All propositions should be true
        tempFormula = ''
        for prop in PropList:
            tempFormula = tempFormula + '\t\t\t ' + prop + ' & \n'

        LTLsubformula = tempFormula
        return LTLsubformula
            
    # A more specific initial condition 
    tempFormula = sentence[:]
    
    # Replace logic operations with TLV convention
    tempFormula = replaceLogicOp(tempFormula)

    # checking that all propositions are 'legal' (in the list of propositions)
    for prop in re.findall('([\w\.]+)',tempFormula):
        if not prop in PropList:
            print 'ERROR(1): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not recognized\n'
            return ''
    
    
    LTLsubformula = '\t\t\t (' + tempFormula + ') & \n'

    return LTLsubformula

def parseSafety(sentence,sensorList,allRobotProp,lineInd):
    ''' This function creates the LTL formula representing a basic safety requirement.
        It takes the sentence, the sensor list and the list of all robot propositions (to check that only 'legal'
        propositions are used and to determine whether it is an environment safety or a robot one)
        and 'lineInd' that indicates which line is being processed.
        Returns a dictionary with 2 keys: 'formula' containing the LTL formula as a string and 'type' containing
        either 'EnvTrans' or 'SysTrans'.
    '''

    
    formulaInfo = {}
    # Initilalizing in case the sentence cannot be parsed
    formulaInfo['formula'] = ''
    formulaInfo['type'] = ''
    
    tempFormula = sentence[:]
    PropList = sensorList + allRobotProp
    
    # Replace logic operations with TLV convention
    tempFormula = replaceLogicOp(tempFormula)

    # checking that all propositions are 'legal' (in the list of propositions)
    for prop in re.findall('([\w\.]+)',tempFormula):
        if not prop in PropList:
            print 'ERROR(2): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not recognized\n'
            formulaInfo['type'] = 'EnvTrans' # arbitrary
            return formulaInfo

        if (prop in sensorList and formulaInfo['type'] == 'SysTrans') or \
           (prop in allRobotProp and formulaInfo['type'] == 'EnvTrans'):
            print 'ERROR(3): Could not parse the sentence in line '+ str(lineInd)+' containing:'
            print sentence
            print 'because both environment and robot propositions are used \n'
            formulaInfo['type'] = 'EnvTrans' # arbitrary
            return formulaInfo

        if prop in sensorList and formulaInfo['type'] == '':
            formulaInfo['type'] = 'EnvTrans'
            # replace every occurrence of the proposition with next(proposition)
            # it is written this way to prevent nesting of 'next' (as with the .replace method)
            tempFormula = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',tempFormula)

        elif prop in allRobotProp and formulaInfo['type'] == '':
            formulaInfo['type'] = 'SysTrans'
            # replace every occurrence of the proposition with next(proposition)
            # it is written this way to prevent nesting of 'next' (as with the .replace method)
            tempFormula = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',tempFormula)

        else:
            # replace every occurrence of the proposition with next(proposition)
            # it is written this way to prevent nesting of 'next' (as with the .replace method)
            tempFormula = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',tempFormula)
    
    
    formulaInfo['formula'] = '\t\t\t [](' + tempFormula + ') & \n'

    return formulaInfo

def parseLiveness(sentence,sensorList,allRobotProp,lineInd):
    ''' This function creates the LTL formula representing a basic liveness requirement.
        It takes the sentence, the sensor list and the list of all robot propositions (to check that only 'legal'
        propositions are used and to determine whether it is an environment safety or a robot one)
        and 'lineInd' that indicates which line is being processed.
        Returns a dictionary with 2 keys: 'formula' containing the LTL formula as a string and 'type' containing
        either 'EnvGoals' or 'SysGoals'.
    '''

    
    formulaInfo = {}
    # Initilalizing in case the sentence cannot be parsed
    formulaInfo['formula'] = ''
    formulaInfo['type'] = ''
    
    tempFormula = sentence[:]
    PropList = sensorList + allRobotProp
    
    # Replace logic operations with TLV convention
    tempFormula = replaceLogicOp(tempFormula)

    # checking that all propositions are 'legal' (in the list of propositions)
    for prop in re.findall('([\w\.]+)',tempFormula):
        if not prop in PropList:
            print 'ERROR(4): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not recognized\n'
            formulaInfo['type'] = 'EnvGoals' # arbitrary
            return formulaInfo

        if (prop in sensorList and formulaInfo['type'] == 'SysGoals') or \
           (prop in allRobotProp and formulaInfo['type'] == 'EnvGoals'):
            print 'ERROR(5): Could not parse the sentence in line '+ str(lineInd)+' containing:'
            print sentence
            print 'because both environment and robot propositions are used \n'
            formulaInfo['type'] = 'EnvGoals' # arbitrary
            return formulaInfo

        if prop in sensorList and formulaInfo['type'] == '':
            formulaInfo['type'] = 'EnvGoals'

        elif prop in allRobotProp and formulaInfo['type'] == '':
            formulaInfo['type'] = 'SysGoals'
    
    
    formulaInfo['formula'] = '\t\t\t []<>(' + tempFormula + ') & \n'

    return formulaInfo


def parseConditional(Condition,ReqFormulaInfo,CondType,sensorList,allRobotProp,lineInd):
    ''' This function creates the LTL formula representing a conditional.
        It takes the condition, the requirement formula (that was already parsed),
        the condition type, and the list of all propositions (to check that only 'legal'
        propositions are used) and 'lineInd' that indicates which line is being processed.
        Returns a dictionary with 2 keys: 'formula' containing the LTL formula as a string and 'type' containing
        the type of the requirement.
    '''

    PropList = sensorList + allRobotProp
    formulaInfo = {}
    # Initilalizing in case the sentence cannot be parsed
    formulaInfo['formula'] = ''
    formulaInfo['type'] = ReqFormulaInfo['type']

    # Getting the subformula encoding the condition
    condFormula = parseCond(Condition,sensorList,allRobotProp,formulaInfo['type'],lineInd)
    if condFormula == '':
        # If could not parse the condition, return
        print 'ERROR(6): Could not parse the condition in line '+ str(lineInd)+'\n'
        return formulaInfo
    
    # Getting the subformula encoding the requirement:
    # First strip and save as formulaStart the '[]' or '[]<>',
    # then remove the '& \n'.
    reqFormulaInd = ReqFormulaInfo['formula'].find('(')
    reqFormula = ReqFormulaInfo['formula'][reqFormulaInd:]
    reqFormula = reqFormula.replace('& \n','')
    formulaStart = ReqFormulaInfo['formula'][:reqFormulaInd]

    # Adding the 2 pieces together
    if CondType == 'IfThen':
        tempFormula = condFormula + ' -> ' + reqFormula
    elif CondType == 'Unless':
        tempFormula = reqFormula  + ' | ' + condFormula
    elif CondType == 'IFF':
        tempFormula = condFormula + ' <-> ' + reqFormula
    else:
        print 'ERROR(7): Could not parse the condition in line '+ str(lineInd)+' because of unknown condition type\n'
        return formulaInfo
    
    
    formulaInfo['formula'] = formulaStart +'(' + tempFormula + ') & \n'

    return formulaInfo


def parseCond(condition,sensorList,allRobotProp,ReqType,lineInd):
    ''' This function creates the LTL formula representing the condition part of a conditional.
        It takes the condition and PropList - a list of propositions (to check that only 'legal'
        propositions are used)and 'lineInd' that indicates which line is being processed.
        Returns the LTL formula as a string.
    '''

    NotFlag = False
    NextFlag = False

    PropList = sensorList + allRobotProp
    tempFormula = ''

    # Flag to indicate whether it is a liveness requirement
    livenessFlag = ReqType == 'EnvGoals' or ReqType == 'SysGoals'
    
    # List of possible condition parts
    regionPastCond = 'the robot was in'+'|'+ 'the robot was not in'+'|' + \
                     'you were in' + '|' + 'you were not in' + '|' + \
                     'it was in' + '|' + 'it was not in' + '|' + \
                     'was in' + '|' + 'was not in' + '|' + \
                     'were in' + '|' + 'were not in' + '|'

    regionCurrCond = 'the robot is in'+'|'+ 'the robot is not in'+'|' + \
                     'you are in' + '|' + 'you are not in' + '|' + \
                     'it is in' + '|' + 'it is not in' + '|' + \
                     'is in' + '|' + 'is not in' + '|' + \
                     'are in' + '|' + 'are not in' + '|'

    sensorPastCond = 'the robot sensed'+'|'+ 'the robot did not sense'+'|' + \
                     'you sensed' + '|' + 'you did not sense' + '|' + \
                     'it sensed' + '|' + 'it did not sense' + '|' + \
                     'sensed' + '|' + 'did not sense' + '|'
  
    sensorCurrCond = 'the robot is sensing'+'|'+ 'the robot is not sensing'+'|' + \
                     'you are sensing' + '|' + 'you are not sensing' + '|' + \
                     'it is sensing' + '|' + 'it is not sensing' + '|' + \
                     'are sensing' + '|' + 'are not sensing' + '|' + \
                     'is sensing' + '|' + 'is not sensing' + '|'

    actionPastCond = 'the robot activated'+'|'+ 'the robot did not activate'+'|' + \
                     'you activated' + '|' + 'you did not activate' + '|' + \
                     'it activated' + '|' + 'it did not activate' + '|' + \
                     'activated' + '|' + 'did not activate' + '|' + \
                     'you were activating' + '|' + 'you were not activating'

    actionCurrCond = 'the robot is activating'+'|'+ 'the robot is not activating'+'|' + \
                     'you are activating' + '|' + 'you are not activating' + '|' + \
                     'it is activating' + '|' + 'it is not activating' + '|' + \
                     'are activating' + '|' + 'are not activating' + '|' + \
                     'is activating' + '|' + 'is not activating' 


    PastCond = regionPastCond + sensorPastCond + actionPastCond
    CurrCond = regionCurrCond + sensorCurrCond + actionCurrCond
    
    possCond = PastCond + '|' + CurrCond 

    condRE = re.compile('('+ possCond + ')',re.IGNORECASE)
    subCond = re.split(condRE,condition)
  

    for subCondition in subCond:

        # If empty string, move on...
        if subCondition == '':
            continue

        # Determine the type of the formula to follow (current)
        # If not a livness requirement, add 'next'
        if (subCondition in CurrCond) and (not livenessFlag) :
            NextFlag = True
            if 'not' in subCondition:
                NotFlag = True
            else:
                NotFlag = False
        # Determine the type of the formula to follow (past)
        # If past OR the requirement is liveness, then do NOT add 'next'
        elif (subCondition in PastCond) or (subCondition in CurrCond):
            NextFlag = False
            if 'not' in subCondition:
                NotFlag = True
            else:
                NotFlag = False
        # This is the body of the subcondition
        else:
            # Add a '!' in front if needed
            if NotFlag:
                subTempFormula = '!(' + subCondition
            else:
                subTempFormula = '(' + subCondition
                     
            # Remove the logic operator (or,and) that connects this subcondition to the next
            # and add it after adding ')', if it exists
            subCondConnect = re.split(' (or|and)\s*$',subTempFormula)
            if len(subCondConnect)==3:
                subTempFormula = subCondConnect[0] + ' ) ' + subCondConnect[1] + ' '
            else:
                subTempFormula = subCondConnect[0] + ' ) '

            # Replace logic operations with TLV convention 
            subTempFormula = replaceLogicOp(subTempFormula)
         
            # checking that all propositions are 'legal' (in the list of propositions)
            # and adding 'next' if needed
            for prop in re.findall('([\w\.]+)',subTempFormula):
                if not prop in PropList:
                    print 'ERROR(8): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not recognized\n'
                    return ''

                if NextFlag and (prop in allRobotProp) and (ReqType == 'EnvTrans') :
                    print 'Warning: In the sentence in line '+ str(lineInd)+' :'
                    print prop + ' is a robot proposition and should be in the past form in an environment safety requirement'
                    print 'The next operator was not added\n'
                    continue

                if NextFlag:
                    # replace every occurrence of the proposition with next(proposition)
                    # it is written this way to prevent nesting of 'next' (as with the .replace method)
                    subTempFormula = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',subTempFormula)

            tempFormula = tempFormula + subTempFormula

                
    
    
    LTLsubformula = '(' + tempFormula + ')'

    return LTLsubformula

def parseEvent(EventProp,SetEvent,ResetEvent,sensorProp,RobotProp,lineInd):
    ''' This function creates the LTL formulas encoding when a proposition should be true and when false.
        This is used as a macro to define 'memory' propositions.
        It takes the proposition, the boolean formulas defining the set and reset events, the propositions
        (to check that only 'legal' propositions are used) and 'lineInd' that indicates which line is being processed.
        Returns the LTL formula as a string.
    '''


    PropList = sensorProp + RobotProp
    
    # Replace logic operations with TLV convention
    SetEvent = replaceLogicOp(SetEvent)
    ResetEvent = replaceLogicOp(ResetEvent)

    # checking that all propositions are 'legal' in the set and reset events, and adding the 'next' operator
    for prop in re.findall('([\w\.]+)',SetEvent):
        if not prop in PropList:
            print 'ERROR(9): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' in the set event is not recognized\n'
            return ''
        else:
            # replace every occurrence of the proposition with next(proposition)
            # it is written this way to prevent nesting of 'next' (as with the .replace method)
            SetEvent = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',SetEvent)

    if ResetEvent.upper()=='FALSE':
        ResetEvent = 'FALSE'
    else:
        for prop in re.findall('([\w\.]+)',ResetEvent):
            if not prop in PropList:
                print 'ERROR(10): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' in the reset event is not recognized\n'
                return ''
            else:
                # replace every occurrence of the proposition with next(proposition)
                # it is written this way to prevent nesting of 'next' (as with the .replace method)
                ResetEvent = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',ResetEvent)

    # Checking the event proposition
    if EventProp in sensorProp:
        print 'ERROR(11): Could not parse the sentence in line '+ str(lineInd)+' because ' + EventProp + ' is a sensor proposition instead of a robot proposition'
        return ''
    elif EventProp in RobotProp:
        pass
    else:
        # Not a single valid robot proposition
        print 'ERROR(12): Could not parse the sentence in line '+ str(lineInd)+' because ' + EventProp + ' is not a valid robot proposition'
        return ''

    # Everything seems to be OK, lets write the formulas
    SetFormula = '\t\t\t ([](( (' + SetEvent + ') & !(' + ResetEvent + ')) -> next(' + EventProp + ')) ) & \n'    
    ResetFormula = '\t\t\t ([](  (' + ResetEvent + ') -> !next(' + EventProp + ')) ) & \n'    
    TrueFormula = '\t\t\t ([](( (' + EventProp + ') & !(' + ResetEvent + ')) -> next(' + EventProp + ')) ) & \n'    
    FalseFormula = '\t\t\t ([](( !(' + EventProp + ') & !(' + SetEvent + ')) -> !next(' + EventProp + ')) ) & \n'    
    
    LTLsubformula = SetFormula + ResetFormula + TrueFormula + FalseFormula

    return LTLsubformula


def replaceLogicOp(formula):
    ''' This function replaces the logic operators with TLV convention.
    '''

    # Replace logic operations with TLV convention
    andRE = re.compile('\\band\\b',re.IGNORECASE)
    orRE = re.compile('\\bor\\b',re.IGNORECASE)
    impliesRE = re.compile('\\bimplies\\b',re.IGNORECASE)
    iffRE = re.compile('\\biff\\b',re.IGNORECASE)
    notRE = re.compile('\\bnot\\b',re.IGNORECASE)
    
    formula = andRE.sub(' & ',formula)
    formula = orRE.sub(' | ',formula)
    formula = impliesRE.sub(' -> ',formula)
    formula = iffRE.sub(' <-> ',formula)
    formula = notRE.sub(' ! ',formula)

    return formula

def replaceRegionName(formula,bitEncode,regionList):
    ''' This function replaces the region names with the appropriate bit encoding.
    '''
    tempFormula = formula[:]
    
    # first replace all 'next' region names with the next encoding
    for nextProp in re.findall('(next\(\w+\))',tempFormula):
        prop = nextProp.replace('next(','')
        prop = prop.replace(')','')
        if prop in regionList:
            ind = regionList.index(prop)
            tempFormula = tempFormula.replace(nextProp, bitEncode['next'][ind])
            # 'replace' is fine here because we are replacing next(region) and that cannot be a partial name
               
    # replace all leftover region names with the current encoding
    for prop in re.findall('(\w+)',tempFormula):
        if prop in regionList:
            ind = regionList.index(prop)
            # replace every occurrence of the proposition with the bit encoding
            # it is written this way to prevent partial word replacements (as with the .replace method)
            tempFormula = re.sub('\s+'+prop, ' '+bitEncode['current'][ind],tempFormula) # if following a space
            tempFormula = re.sub('\('+prop, '('+bitEncode['current'][ind],tempFormula) # if in ()

            #tempFormula = tempFormula.replace(prop, bitEncode['current'][ind])
    
    LTLsubformula = tempFormula 

    return LTLsubformula

def createStayFormula(numBits):
    ''' This function replaces the region names with the appropriate bit encoding.
    '''
    tempFormula = '( (next(s.bit0) <-> s.bit0) '
    
    for bitNum in range(1,numBits):

        # Encoding the string
        tempFormula = tempFormula + '& (next(s.bit'+ str(bitNum) +') <-> s.bit'+ str(bitNum) +') ' 
    
    StayFormula = tempFormula + ')'

    return StayFormula


def bitEncoding(numRegions,numBits):
    ''' This function creates a dictionary that contains the bit encoding for the current
        and next region. Takes number of regions and returns a dictionary with 'current' \
        and 'next' as keys, each containing a list of the respective encodings.
    '''

    # initializing the dictionary
    bitEncode = {}

    # create an encoding of the regions, both current and next
    currBitEnc = []
    nextBitEnc = []
    for num in range(numRegions):
        binary = numpy.binary_repr(num) # regions encoding start with 0
        # Adding zeros
        bitString = '0'*(numBits-len(binary)) + binary

        # Encoding the string
        currTempString = '('
        nextTempString = '('
        for bitNum in range(numBits):
            if bitNum>0:
                currTempString = currTempString + ' & '
                nextTempString = nextTempString + ' & '
            if bitString[bitNum]=='1':
                currTempString = currTempString + 's.bit' + str(bitNum)
                nextTempString = nextTempString + 'next(s.bit' + str(bitNum) + ')'
            if bitString[bitNum]=='0':
                currTempString = currTempString + '!s.bit' + str(bitNum)
                nextTempString = nextTempString + '!next(s.bit' + str(bitNum) + ')'

        currTempString = currTempString + ')'
        nextTempString = nextTempString + ')'
       
        currBitEnc.append(currTempString)
        nextBitEnc.append(nextTempString)


    bitEncode['current'] = currBitEnc
    bitEncode['next'] = nextBitEnc

    return bitEncode

