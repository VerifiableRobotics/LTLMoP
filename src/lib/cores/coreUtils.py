
import math, re, sys, random, os, subprocess, time
from copy import copy
from logic import to_cnf
from multiprocessing import Pool




def conjunctsToCNF(conjuncts, propList):
    
    propListNext = map(lambda s: 'next_'+s, propList)
    
    props = {propList[x]:x+1 for x in range(0,len(propList))}
    propsNext = {propListNext[x]:len(propList)+x+1 for x in range(0,len(propListNext))}
    mapping = {conjuncts[x]:[] for x in range(0,len(conjuncts))}
    
    cnfClauses = []
    transClauses = []
    goalClauses = []
    n = 0
    p = len(props)+len(propsNext)       
    
    pool = Pool()
    #print "STARTING CNF MAP"
    allCnfs = pool.map(lineToCnf, conjuncts, chunksize = 1)   
    #print "ENDING CNF MAP"
        
    for cnf, lineOld in zip(allCnfs,conjuncts):
      if cnf is not None: 
        allClauses = cnf.split("&");
        #associate original conjuncts with CNF clauses
        for clause in allClauses:    
            clause = re.sub('[()]', '', clause)   
            clause = re.sub('[|]', '', clause)           
            clause = re.sub('~', '-', clause)    
            #replace prop names with var numbers
            for k in propsNext.keys():
                clause = re.sub(k,str(propsNext[k]), clause)
            for k in props.keys():
                    clause = re.sub(k,str(props[k]), clause)   
                #add trailing 0   
            if "<>" in lineOld:
                goalClauses.append(clause.strip()+" 0\n")
            elif "[]" in lineOld:
                #isTrans[lineOld]:
                transClauses.append(clause.strip()+" 0\n")
                cnfClauses.append(clause.strip()+" 0\n")                                
            else:
                cnfClauses.append(clause.strip()+" 0\n")         
            
        if not "<>" in lineOld:
            mapping[lineOld].extend(range(n+1,n+1+len(allClauses)))    
            n = n + len(allClauses)
    
    
        
    # Create disjunction of goal over all the time steps         
    """finalDisj = ""     
    
    firstDisj = True        
    for i in range(1,depth+2):   
        newClause = ""                                           
        firstConj = True
        for clause in goalClauses:
             if not firstConj:
                 newClause= newClause + " & "
             f = True
             for c in clause.split():
                 intC = int(c)
                 if intC is not 0:                    
                  if not f:
                     newClause= newClause + " | " + str(cmp(intC,0)*(abs(intC)+len(props)*(i-1)))
                  else:
                     newClause= newClause + str(cmp(intC,0)*(abs(intC)+len(props)*(i-1)))
                 f = False
             firstConj = False
        if finalDisj is not "":
            finalDisj = finalDisj + "|" 
        finalDisj = finalDisj + newClause
        firstDisj = False

        

    
    finalConj = []
    if finalDisj is not "":
        for c in str(to_cnf(finalDisj)).split('&'):
            finalConj.append(re.sub('[\(\)|&]*','',c) + " 0\n")
    
            
            # add disjuncts to the goal clause (goal is satisfied in at least one of the time steps)
            # assumes goals all contain <> on line (so no line breaks within goals)
            if "<>" in line:
                for v in mapping[line]:
                    newDisjuncts = ""
                    currGoals = cnfClauses[v-1].split()
                    numVarsInGoal = (len(currGoals) - 1)/i
                    
                    for c in currGoals[-(numVarsInGoal+1):-1]:
                        intC = int(c)
                        if intC is not 0:                    
                            newDisjuncts= newDisjuncts + str(cmp(intC,0)*(abs(intC)+len(props)*(i))) +" "
                            
                    # adding disjuncts here
                    cnfClauses[v-1] = newDisjuncts + cnfClauses[v-1]                               
"""                

    
    
    #for line in conjuncts:        
        #if "<>" in line:
            #mapping[line] = range(n+1,n+len(finalConj)+1)                       
    
    #n = n + len(finalConj)
    #cnfClauses.extend(finalConj)

 
            

        
        
    """#write CNFs to file        
    open(outFilename, 'w').close()
    output = open(outFilename, 'a')
    output.write("p cnf "+str(p)+" "+str(n)+"\n")
    output.writelines(cnfClauses)
    output.close()
    """
    #dimacs = "p cnf "+str(p)+" "+str(n)+"\n" + "".join(cnfClauses)
    
    """for line in conjuncts:        
        if "<>" in line:
            mapping[line] = range(n+1,n+len(goalClauses)+1)   
            """
                        
    return mapping, cnfClauses, transClauses, goalClauses
    
    #for i in range(0,depth):
    #        for k in propsNext.keys():
    #                print str(propsNext[k]+len(propsNext)*(i)) + " " + k
    #        for k in props.keys():
    #                print str(props[k]+len(props)*(i)) + " " + k


def cnfToConjuncts(cnfIndices, mapping):
    conjuncts = []
    i = 0
    for k in mapping.keys():
        i = i + 1
        if not set(mapping[k]).isdisjoint(cnfIndices):
            conjuncts.append(k)     
            #print k , (set(mapping[k]).intersection(cnfIndices))
    return conjuncts


def lineToCnf(line):
            
        lineOld = line
        line = re.sub('[\t\n]*','',line)            
        line = re.sub('s\.','',line)
        line = re.sub('e\.','',line)   
        line = re.sub(r'(next\(\s*!)', r'(!next_', line)         
        line = re.sub(r'(next\()', r'(next_', line)         
        line = re.sub('\<\>','',line)  
        line = re.sub('\[\]','',line)  
        line = line.strip()
        #trailing &
        line = re.sub('&\s*$','',line)  
        if line!='':
            
            line = re.sub('!', '~', line)
            #line = re.sub('&\s*\n', '', line)
            line = re.sub('[\s]+', ' ', line)        
            line = re.sub('\<-\>', '<=>', line)
            line = re.sub('->', '>>', line)
            line = line.strip() 
            cnf = str(to_cnf(line))
            return cnf
        else:
            return None
        
def findGuiltyClausesWrapper(x):        
        return findGuiltyClauses(*x)
        
def findGuiltyClauses(cmd, depth, numProps, init, trans, goals, mapping, conjuncts): 
        transClauses = []
        #Duplicating transition clauses for depth greater than 1         
        numOrigClauses = len(trans)  
        #the depth tells you how many time steps of trans to use
        #depth 0 just checks init with goals
        #p = 0
        for i in range(1,depth+1):
                    transClausesNew = []
                    for clause in trans:
                        newClause = ""
                        for c in clause.split():
                            intC = int(c)
                            newClause= newClause + str(cmp(intC,0)*(abs(intC)+numProps*i)) +" "
                            #p = max(p, (abs(intC)+numProps*i))
                        newClause=newClause+"\n"
                        transClausesNew.append(newClause)
                    j = 0    
                    for line in conjuncts:
                        if "[]" in line and "<>" not in line:                      
                            numVarsInTrans = (len(mapping[line]))/(i+1)
                            mapping[line].extend(map(lambda x: x+numOrigClauses, mapping[line][-numVarsInTrans:]))
                            j = j + 1
                    transClauses.extend(transClausesNew)
                    print "A",len(transClauses)
        #create goal clauses
        dg = map(lambda x: ' '.join(map(lambda y: str(cmp(int(y),0)*(abs(int(y))+numProps*(depth))), x.split())) + '\n', goals)
        #for g in dg:
            #for c in g.split():                            
                #p = max(p, abs(int(c)))
                
                                
        n = len(transClauses) + len(init)
        for line in conjuncts:
            if "<>" in line:
                mapping[line] = range(n+1,n+len(goals)+1)
                
        #combine the clauses
        cnfs = init + transClauses + dg            
        
    
        #create picomus input
        
        #precompute p and n
        #p =  (depth+1)*(numProps*2)
        p = (depth+2)*(numProps)        
        #n = len(cnfs)  
        n = (depth)*(len(trans)) + len(init) + len(goals)
        
        input = "p cnf "+str(p)+" "+str(n)+"\n" + "".join(cnfs)               
            
        
                
        #find minimal unsatisfiable core by calling picomus
        if cmd is None:
            return (False, False, [], "")        
 
        subp = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=False)                                            
        output = subp.communicate(input)[0]                                         
                                                                                      
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__

        """#this is the BMC part: keep adding cnf clauses from the transitions until the spec becomes unsatisfiable
            if "UNSATISFIABLE" in output or depth >= maxDepth:
                    break
            depth = depth +1
            """
        if "UNSATISFIABLE" in output:
            print "Unsatisfiable core found at depth ", depth
        elif "SATISFIABLE" in output:
            print "Satisfiable at depth ", depth
        else:
            print "ERROR", output
            
                    
            
            
        """#Write output to file (mainly for debugging purposes)
            satFileName = self.proj.getFilenamePrefix()+".sat"
            outputFile = open(satFileName,'w')
            outputFile.write(output)
            outputFile.close()
            """
            
    
        
        """cnfIndices = []
        for line in output.split('\n'):
                if re.match('^v', line):
                    index = int(line.strip('v').strip())
                    if index!=0:
                        cnfIndices.append(index)
            """
        #pythonified the above
        #get indices of contributing clauses
        cnfIndices = filter(lambda y: y!=0, map((lambda x: int(x.strip('v').strip())), filter(lambda z: re.match('^v', z), output.split('\n'))))
        
        #get corresponding LTL conjuncts
        guilty = cnfToConjuncts(cnfIndices, mapping)
            
        return guilty
        
        
        
