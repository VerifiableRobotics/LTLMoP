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
    
    regionGroups = {}
    sensorGroups = {}
    
    #Open CFG file
    grammarFile = open('grammar1.fcfg')
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
    groupDefPattern = 'group (\w+) is (?:('
    groupDefPattern += '),? ?|('.join(regionList)
    groupDefPattern += '),? ?)+'
    r_groupDef = re.compile(groupDefPattern)
    
    #Generate regular expression to match sentences defining sensor groups
    #Resultant expression is: 'group (\w+) is (?:(region1),? ?|(region2),? ?|(region3),? ?)+'
    sensorGroupDefPattern = 'sensor group (\w+) is (?:('
    sensorGroupDefPattern += '),? ?|('.join(sensorList)
    sensorGroupDefPattern += '),? ?)+'
    r_sensorGroupDef = re.compile(sensorGroupDefPattern)
    
    #Generate NLTK feature grammar object from grammar string
    grammar = nltk.grammar.parse_fcfg(grammarText)
    
    lineInd = 0
    
    #Iterate over the lines in the file
    for line in text.split("\n"):

        lineInd += 1

        #If it is an empty line, ignore it
        if re.search('^(\s*)$',line):
            continue
            
        #If the sentence is a comment, ignore it
        if CommentRE.search(line):
            continue

        #If there is a newline at the end, remove it
        if re.search('\n$',line):
            line = re.sub('\n$','',line)
        
        #Examine input line to find any region group definitons
        m_groupDef = r_groupDef.match(inputString)
        if m_groupDef:
            #Add semantics of group to our grammar string
            groupName = m_groupDef.groups()[0]
            grammarText += '\nGROUP[SEM=<' + groupName + '>] -> \'' + groupName + '\''
            #Add specified regions to our dictionary of region groups
            regionGroups[groupName] = filter(lambda x: x != None, m_groupDef.groups()[1:])
            print 'Groups updated: ' + str(regionGroups)
            #Re-compile grammar
            grammar = nltk.grammar.parse_fcfg(grammarText)
            continue
        
        #Examine input line to find any sensor group definitions
        m_sensorGroupDef = r_sensorGroupDef.match(inputString)
        if m_sensorGroupDef:
            #Add semantics of group to our grammar string
            groupName = m_sensorGroupDef.groups()[0]
            grammarText += '\nSENSORGROUP[SEM=<' + groupName + '>] -> \'' + groupName + '\''
            #Add specified sensors to out dictionary of sensor groups
            sensorGroups[groupName] = filter(lambda x: x != None, m_sensorGroupDef.groups()[1:])
            print 'Sensor groups updated: ' + str(sensorGroups)
            #Re-compile grammar
            grammar = nltk.grammar.parse_fcfg(grammarText)
            continue
        
        #Parse input line using grammar; the result here is a collection
        # of pairs of syntax trees and semantic representations in a
        # prefix first-order-logic syntax
        result = nltk.sem.batch_interpret([inputString], grammar, u'SEM', 2)[0]
        
        nTrees = 0;
        formulaeFound = []
        #Iterate over all parse trees found
        for (syntree, semrep) in result:
            semstring = str(semrep)
            
            #We are not interested multiple parse trees that
            # produce the same LTL formula, so skip them
            if semstring in formulaeFound
                continue
            nTrees += 1
            
            #Expand 'stay' phrases
            semstring = parseStay(semstring)
            #Expand region groups, 'any' and 'all'
            semstring = parseGroupAny(semstring, syntree)
            semstring = parseGroupAll(semstring, syntree)
            #Expand memory propositions
            semstring = parseMemory(semstring, syntree)
            semstring = parseToggle(semstring, syntree)
            
            formulaeFound.append(semstring)
            
            #Convert formula from prefix FOL to infix LTL and add it to
            # the appropriate section of the specification
            stringLTL = prefixFOL2InfixLTL(semstring)
            spec[syntree.node['SPEC']] += stringLTL + '&'
            linemap[syntree.node['SPEC']].append(lineInd)
            LTL2LineNo[stringLTL] = lineInd
            
            break #For now we'll only look at the first result
            #TODO: Need to display and resolve ambiguity in the future
        
        if nTrees == 0:
            print 'No valid parse found!'
            failed = True
    
    #Set all empty subformulas to TRUE, and removing last & in 'EnvGoals' and 'SysGoals'
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
        
    return spec,linemap,failed,LTL2LineNo
            
def parseStay(semstring):
    def appendStayClause(ind):
        if ind == len(regions) - 1:
            return 'Iff(Next('+regions[ind]+'),'+regions[ind]+')'
        else:
            return 'And(Iff(Next('+regions[ind]+'),'+regions[ind]+'),'+appendStayClause(ind+1)+')'
    if semstring.find('$stay') != -1:       
        stay = appendStayClause(0)
        return semstring.replace('$stay',stay)
    else:
        return semstring

def parseGroupAll(semstring, syntree):
    def appendAllClause(semstring, ind, groupRegions):
        if ind == len(groupRegions) - 1:
            return re.sub('\$All\(\w+\)',groupRegions[ind],semstring)
        else:
            return 'And('+re.sub('\$All\(\w+\)',groupRegions[ind],semstring)+','+appendAllClause(semstring, ind+1, groupRegions)+')'
    while semstring.find('$All') != -1:
        groupName = re.search('\$All\((\w+)\)',semstring).groups()[0]
        if groupName in regionGroups:
            groupItems = regionGroups[groupName]
        elif groupName in sensorGroups:
            groupItems = sensorGroups[groupName]
        semstring = appendAllClause(semstring, 0, groupItems)
    return semstring
    
def parseGroupAny(semstring, syntree):
    def appendAnyClause(ind, groupRegions):
        if ind == len(groupRegions) - 1:
            return groupRegions[ind]
        else:
            return 'Or('+groupRegions[ind]+','+appendAnyClause(ind+1, groupRegions)+')'
    while semstring.find('$Any') != -1:
        groupName = re.search('\$Any\((\w+)\)',semstring).groups()[0]
        if groupName in regionGroups:
            groupItems = regionGroups[groupName]
        elif groupName in sensorGroups:
            groupItems = sensorGroups[groupName]
        anyClause = appendAnyClause(0, groupItems)
        semstring = re.sub('\$Any\('+groupName+'\)', anyClause, semstring)
    return semstring
        
def parseMemory(semstring, syntree):
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
    
def parseToggle(semstring, syntree):
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

def prefixFOL2InfixLTL(prefixString):
    andGroups = re.match('And\((.*),(.*)\)', prefixString)    
    orGroups = re.match('Or\((.*),(.*)\)', prefixString)
    notGroups = re.match('Not\((.*)\)', prefixString)
    nextGroups = re.match('Next\((.*)\)', prefixString)
    globallyGroups = re.match('Glob\((.*)\)', prefixString)
    globallyFinallyGroups = re.match('GlobFin\((.*)\)', prefixString)
    impGroups = re.match('Imp\((.*),(.*)\)',prefixString)
    
    if andGroups:
        return '(' + prefixFOL2InfixLTL(andGroups.groups()[0]) + ' & ' + prefixFOL2InfixLTL(andGroups.groups()[1]) + ')'
    elif orGroups:
        return '(' + prefixFOL2InfixLTL(orGroups.groups()[0]) + ' | ' + prefixFOL2InfixLTL(orGroups.groups()[1]) + ')'
    elif notGroups:
        return '!(' + prefixFOL2InfixLTL(notGroups.groups()[0]) + ')'
    elif nextGroups:
        return 'next(' + prefixFOL2InfixLTL(nextGroups.groups()[0]) + ')'
    elif globallyGroups:
        return '[](' + prefixFOL2InfixLTL(globallyGroups.groups()[0]) + ')'
    elif globallyFinallyGroups:
        return '[]<>(' + prefixFOL2InfixLTL(globallyFinallyGroups.groups()[0]) + ')'
    elif impGroups:
        return '(' + prefixFOL2InfixLTL(impGroups.groups()[0]) + ' -> ' + prefixFOL2InfixLTL(impGroups.groups()[1]) + ')'
    else:
        return prefixString
