""" 
    ===================================================
    parseSpec.py - Structured English to LTL Translator 
    ===================================================
    
    Module that parses a set of structured English sentences into the
    corresponding LTL subformulas using a context-free grammar
"""

import re
import copy
import nltk

def writeSpec(text, sensorList, regionList, robotPropList):
    ''' This function creates the Spec dictionary that contains the parsed LTL
        subformulas. It takes the text that contains the structured English,
        the list of sensor propositions, the list containing
        the region names and the list of robot propositions (other than regions). 
    '''
    
    #Begin optimistically
    failed = False
    
    #Initialize the dictionary
    spec = {}
    spec['EnvInit']= ''
    spec['EnvTrans']= ''
    spec['EnvGoals']= ''
    spec['SysInit']= ''
    spec['SysTrans']= ''
    spec['SysGoals']= ''

    linemap = {}
    linemap['EnvInit']= []
    linemap['EnvTrans']= []
    linemap['EnvGoals']= []
    linemap['SysInit']= []    
    linemap['SysTrans']= []
    linemap['SysGoals']= []
    
    LTL2LineNo = {}

    allProps = sensorList + regionList + robotPropList
    
    #Initialize dictionaries mapping group names to lists of groups
    regionGroups = {}
    sensorGroups = {}
    actionGroups = {}
    allGroups = {}
    
    #Initialize dictionary mapping propositions to sets of 'corresponding' propositions
    correlations = {}
    
    #Open CFG file
    # TODO MAKE INDEP of path
    grammarFile = open('structuredEnglish.fcfg')
    grammarText = grammarFile.read()
    
    #Add production rules for region names to our grammar string
    for region in regionList:
        grammarText += '\nREGION[SEM=<'+region+'>] -> \''+region+'\''
    
    #Add production rules for action names to our grammar string
    for action in robotPropList:
        grammarText += '\nACTION[SEM=<'+action+'>] -> \''+action+'\''
    
    #Add production rules for sensor names to our grammar string
    for sensor in sensorList:
        grammarText += '\nSENSOR[SEM=<'+sensor+'>] -> \''+sensor+'\''
    
    #Generate regular expression to match sentences defining region groups
    #Resultant expression is: 'group (\w+) is (?:(region1),? ?|(region2),? ?|(region3),? ?)+'
    groupDefPattern = '\s*group (\w+) is (?:('
    groupDefPattern += '),? ?|('.join(regionList)
    groupDefPattern += '),? ?)+'
    r_groupDef = re.compile(groupDefPattern, re.I)
    
    #Generate regular expression to match sentences defining sensor groups
    #Resultant expression is: 'sensor group (\w+) is (?:(sensor1),? ?|(sensor2),? ?|(sensor3),? ?)+'
    sensorGroupDefPattern = '\s*group (\w+) is (?:('
    sensorGroupDefPattern += '),? ?|('.join(sensorList)
    sensorGroupDefPattern += '),? ?)+'
    r_sensorGroupDef = re.compile(sensorGroupDefPattern, re.I)
    
    #Generate regular expression to match sentences defining action groups
    #Resultant expression is: 'group (\w+) is (?:(action1),? ?|(action2),? ?|(action3),? ?)+'
    actionGroupDefPattern = '\s*group (\w+) is (?:('
    actionGroupDefPattern += '),? ?|('.join(robotPropList)
    actionGroupDefPattern += '),? ?)+'
    r_actionGroupDef = re.compile(actionGroupDefPattern)
    
    #Generate regular expression to match sentences defining correlations
    correlationDefPattern = '((?:[_\w]+,? )+)correspond to((?:,? [_\w]+)+)'
    r_correlationDef = re.compile(correlationDefPattern)
    
    #Generate NLTK feature grammar object from grammar string
    grammar = nltk.grammar.parse_fcfg(grammarText)
    
    lineInd = 0
    
    #Iterate over the lines in the file
    for line in text.split("\n"):
        
        line = line.lower()
        lineInd += 1

        #If it is an empty line, ignore it
        if re.search('^(\s*)$',line):
            continue
            
        #If the sentence is a comment, ignore it
        if re.search('^\s*#',line):
            continue

        #If there is a newline at the end, remove it
        if re.search('\n$',line):
            line = re.sub('\n$','',line)
        
        #Examine input line to find any region group definitons
        m_groupDef = r_groupDef.match(line)
        if m_groupDef:
            #Add semantics of group to our grammar string
            groupName = m_groupDef.groups()[0]
            grammarText += '\nGROUP[SEM=<' + groupName + '>] -> \'' + groupName + '\''
            #Add specified regions to our dictionary of region groups
            regionGroups[groupName] = filter(lambda x: x != None, m_groupDef.groups()[1:])
            allGroups[groupName] = regionGroups[groupName]
            print 'Groups updated: ' + str(regionGroups)
            #Re-compile grammar
            grammar = nltk.grammar.parse_fcfg(grammarText)
            continue
        
        #Examine input line to find any sensor group definitions
        m_sensorGroupDef = r_sensorGroupDef.match(line)
        if m_sensorGroupDef:
            #Add semantics of group to our grammar string
            groupName = m_sensorGroupDef.groups()[0]
            grammarText += '\nSENSORGROUP[SEM=<' + groupName + '>] -> \'' + groupName + '\''
            #Add specified sensors to out dictionary of sensor groups
            sensorGroups[groupName] = filter(lambda x: x != None, m_sensorGroupDef.groups()[1:])
            allGroups[groupName] = sensorGroups[groupName]
            print 'Sensor groups updated: ' + str(sensorGroups)
            #Re-compile grammar
            grammar = nltk.grammar.parse_fcfg(grammarText)
            continue
            
        #Examine input line to find any action group definitions
        m_actionGroupDef = r_actionGroupDef.match(line)
        if m_actionGroupDef:
            #Add semantics of group to our grammar string
            groupName = m_actionGroupDef.groups()[0]
            grammarText += '\nACTIONGROUP[SEM=<' + groupName + '>] -> \'' + groupName + '\''
            #Add specified sensors to out dictionary of sensor groups
            actionGroups[groupName] = filter(lambda x: x != None, m_actionGroupDef.groups()[1:])
            allGroups[groupName] = actionGroups[groupName]
            print 'Action groups updated: ' + str(actionGroups)
            #Re-compile grammar
            grammar = nltk.grammar.parse_fcfg(grammarText)
            continue
            
        #Examine input line to find any correlation definitions
        m_correlationDef = r_correlationDef.match(line)
        if m_correlationDef:
            keyItems = filter(lambda x: x != '', re.split(' |, |,', m_correlationDef.group(1)))
            valueItems = filter(lambda x: x != '', re.split(' |, |,', m_correlationDef.group(2)))
            if len(keyItems) != len(valueItems):
                print 'Error: Correlations must be made in pairs!'
                failed = True
                continue
            for prop in keyItems + valueItems:
                if prop not in allProps:
                    print('Error: Could not resolve proposition \'' + prop + '\'')
                    failed = True
                    continue
            #Add specified correlation(s) to dictionary of correlations
            nPairs = len(keyItems)
            for ind in range(0,nPairs):
                if keyItems[ind] in correlations:
                    correlations[keyItems[ind]].append(valueItems[ind])
                else:
                    correlations[keyItems[ind]] = [valueItems[ind]]
                print 'Correlations updated: ' + keyItems[ind] + ' corresponds to ' + valueItems[ind]
            continue
        
        #Parse input line using grammar; the result here is a collection
        # of pairs of syntax trees and semantic representations in a
        # prefix first-order-logic syntax
        result = nltk.sem.batch_interpret([line], grammar, u'SEM', 2)[0]
        
        nTrees = 0;
        formulaeFound = []
        #Iterate over all parse trees found
        for (syntree, semrep) in result:
            semstring = str(semrep)
            
            #We are not interested multiple parse trees that
            # produce the same LTL formula, so skip them
            if semstring in formulaeFound:
                continue
            nTrees += 1
            
            #Expand 'corresponding' phrases
            semstring = parseCorresponding(semstring, correlations, allGroups)
            #Expand 'stay' phrases
            semstring = parseStay(semstring, regionList)
            #Expand groups, 'each', 'any', and 'all'
            semstring = parseGroupEach(semstring, allGroups)
            semstring = parseGroupAny(semstring, allGroups)
            semstring = parseGroupAll(semstring, allGroups)
            #Expand memory propositions
            semstring = parseMemory(semstring)
            semstring = parseToggle(semstring)
            
            formulaeFound.append(semstring)
            
            #Convert formula from prefix FOL to infix LTL and add it to
            # the appropriate section of the specification
            stringLTL = prefix2infix(semstring)
            spec[syntree.node['SPEC']] += stringLTL + ' & \n'
            linemap[syntree.node['SPEC']].append(lineInd)
            LTL2LineNo[stringLTL] = lineInd
            
            break #For now we'll only look at the first result
            #TODO: Need to display and resolve ambiguity in the future
        
        if nTrees == 0:
            print 'Error: No valid parse found for: ' + line
            failed = True
    
    #Set all empty subformulas to TRUE, and removing last & in 'EnvGoals' and 'SysGoals'
    if spec['EnvInit'] == '':
        spec['EnvInit'] = 'TRUE & \n'
    if spec['EnvTrans'] == '':
        spec['EnvTrans'] = '[](TRUE) & \n'
    if spec['EnvGoals'] == '':
        spec['EnvGoals'] = '[]<>(TRUE) \n'
    else:
        # remove last &
        spec['EnvGoals'] = re.sub('& \n$','\n',spec['EnvGoals'])

    # No need to change anything in SysTrans,
    # since the transition relation is encoded anyway
    if spec['SysGoals'] == '':
        spec['SysGoals'] = '[]<>(TRUE)'
    else:
        # remove last &
        spec['SysGoals'] = re.sub('& \n$','\n',spec['SysGoals'])
        
    return spec,linemap,failed,LTL2LineNo
            
def parseStay(semstring, regions):
    def appendStayClause(ind):
        if ind == len(regions) - 1:
            return 'Iff(Next('+regions[ind]+'),'+regions[ind]+')'
        else:
            return 'And(Iff(Next('+regions[ind]+'),'+regions[ind]+'),'+appendStayClause(ind+1)+')'
    if semstring.find('$Stay') != -1:       
        stay = appendStayClause(0)
        return semstring.replace('$Stay',stay)
    else:
        return semstring

def parseGroupAll(semstring, allGroups):
    def appendAllClause(semstring, ind, groupItems):
        if ind == len(groupItems) - 1:
            return re.sub('\$All\(\w+\)',groupItems[ind],semstring)
        else:
            return 'And('+re.sub('\$All\(\w+\)',groupItems[ind],semstring)+','+appendAllClause(semstring, ind+1, groupItems)+')'
    while semstring.find('$All') != -1:
        groupName = re.search('\$All\((\w+)\)',semstring).groups()[0]
        if groupName in allGroups:
            semstring = appendAllClause(semstring, 0, allGroups[groupName])
        else:
            print('Error: Could not resolve group '+groupName)
    return semstring
    
def parseGroupAny(semstring, allGroups):
    def appendAnyClause(ind, groupItems):
        if ind == len(groupItems) - 1:
            return groupItems[ind]
        else:
            return 'Or('+groupItems[ind]+','+appendAnyClause(ind+1, groupItems)+')'
    while semstring.find('$Any') != -1:
        groupName = re.search('\$Any\((\w+)\)',semstring).groups()[0]
        if groupName in allGroups:
            anyClause = appendAnyClause(0, allGroups[groupName])
            semstring = re.sub('\$Any\('+groupName+'\)', anyClause, semstring)
        else:
            print('Error: Could not resolve group '+groupName)
    return semstring
    
def parseGroupEach(semstring, allGroups):
    while semstring.find('$Each') != -1:
        groupName = re.search('\$Each\((\w+)\)',semstring).group(1)
        if groupName in allGroups:
            newSentences = []
            for groupItem in allGroups[groupName]:
                newSentences.append(re.sub('\$Each\('+groupName+'\)',groupItem,semstring))
            semstring = 'And('+',And('.join(newSentences)+')'*(len(newSentences)-1)
        else:
            print('Error: Could not resolve group '+groupName)
    return semstring
        
def parseMemory(semstring):
    if semstring.find('$Mem') != -1:
        m_Mem = re.search('\$Mem\((.*),(.*),(.*)\)',semstring)
        phi_m = m_Mem.groups()[0]
        phi_s = m_Mem.groups()[1]
        phi_r = m_Mem.groups()[2]
        setClause = 'Glob(Imp(And('+phi_s+',Not('+phi_r+')),Next('+phi_m+')))'
        resetClause = 'Glob(Imp('+phi_r+',Not(Next('+phi_m+'))))'
        holdOnClause = 'Glob(Imp(And('+phi_m+',Not('+phi_r+')),Next('+phi_m+')))'
        holdOffClause = 'Glob(Imp(And(Not('+phi_m+'),Not('+phi_s+')),Not(Next('+phi_m+'))))'
        if phi_r == 'false':
            #Generate memory statement with no reset
            semstring = 'And('+setClause+','+holdOnClause+')'
        else:
            #Generate memory statement with reset
            semstring = 'And('+setClause+',And('+resetClause+',And('+holdOnClause+','+holdOffClause+')))'
    return semstring
    
def parseToggle(semstring):
    if semstring.find('$Tog') != -1:
        m_Tog = re.search('\$Tog\((.*),(.*)\)',semstring)
        phi_m = m_Tog.groups()[0]
        phi_t = m_Tog.groups()[1]
        turnOffClause = 'Glob(Imp(And('+phi_m+','+phi_t+'),Not(Next('+phi_m+'))))'
        turnOnClause = 'Glob(Imp(And(Not('+phi_m+'),'+phi_t+'),Next('+phi_m+')))'
        holdOnClause = 'Glob(Imp(And('+phi_m+',Not('+phi_t+')),Next('+phi_m+')))'
        holdOffClause = 'Glob(Imp(And(Not('+phi_m+'),Not('+phi_t+')),Not(Next('+phi_m+'))))'
        semstring = 'And('+turnOnClause+',And('+turnOffClause+',And('+holdOnClause+','+holdOffClause+')))'
    return semstring
    
def parseCorresponding(semstring, correlations, allGroups):
    if semstring.find('$Corr') != -1:
        m_Any = re.search('\$Any\((\w+)\)',semstring)
        m_Each = re.search('\$Each\((\w+)\)',semstring)
        m_Corr = re.search('\$Corr\((\w+)\)',semstring)
        indexGroupName = ''
        indexGroup = []
        indexPhrase = ''
        if m_Any:
            indexGroupName = m_Any.groups()[0]
            indexPhrase = m_Any.group(0)
        elif m_Each:
            indexGroupName = m_Each.groups()[0]
            indexPhrase = m_Each.group(0)
        else:
            print('Error: \'corresponding\' must be be preceded by an \'any\' or \'each\' quantifier')
            failed = True
            return ''
        indexGroup = allGroups[indexGroupName]
        #Iterate over items in indexGroup, replacing each 'corresponding' with the 
        # intersection of items correlated with indexItem and items in the relevant group
        newSentences = []
        for indexItem in indexGroup:
            if indexItem not in correlations:
                print('Error: no correlation found for item \'' + indexItem + '\' in group \'' + indexGroupName + '\'')
                return ''
            for valueGroupName in m_Corr.groups():
                valueGroup = allGroups[valueGroupName]
                for corrItem in correlations[indexItem]:
                    if corrItem in valueGroup:
                        newSentences.append(semstring.replace(indexPhrase,indexItem).replace('$Corr('+valueGroupName+')',corrItem))
                        break
        if len(newSentences) == 0:
            print 'Error: cannot resolve correlations in \'' + semstring + '\''
            return ''
        #Build conjunction out of all generated sentences
        if len(newSentences) == 1:
            semstring = newSentences[0]
        else:
            semstring = 'And(' + ',And('.join(newSentences[0:-1]) + ',' + newSentences[-1] + ')'*(len(newSentences)-1)
        
    return semstring

def prefix2infix(prefixString):
    inList = re.split('[(),]+',prefixString)
    
    def opReduce(inList, index):
        if inList[index] == 'And':
            return '(' + opReduce(inList, index + 1) + ' & ' + opReduce(inList, index + 2) + ')'
        elif inList[index] == 'Or':
            return '(' + opReduce(inList, index + 1) + ' | ' + opReduce(inList, index + 2) + ')'
        elif inList[index] == 'Imp':
            return '(' + opReduce(inList, index + 1) + ' -> ' + opReduce(inList, index + 2) + ')'
        elif inList[index] == 'Iff':
            return '(' + opReduce(inList, index + 1) + ' <-> ' + opReduce(inList, index + 2) + ')'
        elif inList[index] == 'Not':
            return '!(' + opReduce(inList, index + 1) + ')'
        elif inList[index] == 'Next':
            return 'next(' + opReduce(inList, index + 1) + ')'
        elif inList[index] == 'Glob':
            return '[](' + opReduce(inList, index + 1) + ')'
        elif inList[index] == 'GlobFin':
            return '[]<>(' + opReduce(inList, index + 1) + ')'
        else:
            return inList[index]
    
    return opReduce(inList, 0)
