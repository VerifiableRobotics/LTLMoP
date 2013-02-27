
import math, re, sys, random, os, subprocess, time
from copy import copy, deepcopy
from logic import to_cnf
from multiprocessing import Pool
import threading
import itertools





def conjunctsToCNF(topoInit, conjunctsMinusTopo, propList):
    
    conjuncts = topoInit + conjunctsMinusTopo         
    
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
    print "STARTING CNF MAP"
    allCnfs = pool.map(lineToCnf, conjuncts, chunksize = 1)   
    #allCnfs = map(lineToCnf, conjuncts)   
    print "ENDING CNF MAP"
    pool.terminate()
    
    cnfMapping = {line:cnf.split("&") for cnf, line in zip(allCnfs,conjuncts) if cnf}  
      
    for cnf, lineOld in zip(allCnfs,conjuncts):
      if cnf: 
        allClauses = cnf.split("&");
        #associate original conjuncts with CNF clauses
        for clause in allClauses:    
            clause = re.sub('[()]', '', clause)   
            clause = re.sub('[|]', '', clause)           
            clause = re.sub('~', '-', clause)    
            #replace prop names with var numbers
            for k in propsNext.keys():
                clause = re.sub("\\b"+k+"\\b",str(propsNext[k]), clause)
            for k in props.keys():
                    clause = re.sub("\\b"+k+"\\b",str(props[k]), clause)   
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
                        
    return mapping, cnfMapping, cnfClauses, transClauses, goalClauses
    

def cnfToConjuncts(cnfIndices, mapping, cnfMapping):
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
    
def subprocessReadThread(fd, out):
            for line in fd:                                                                              
               out.append(line) 
                
        
def findGuiltyLTLConjunctsWrapper(x):        
        return findGuiltyLTLConjuncts(*x)


        
def findGuiltyLTLConjuncts(cmd, depth, numProps, init, trans, goals, mapping,  cnfMapping, conjuncts, ignoreDepth): 
        mapping = deepcopy(mapping)
        #precompute p and n
        #p =  (depth+1)*(numProps*2)
        p = (depth+2)*(numProps)        
        #n = len(cnfs)  
        n = (depth)*(len(trans)) + len(init) + len(goals)
        if ignoreDepth == 0:
            ignoreBound = 0
        else:
            ignoreBound = len(init) + ignoreDepth*len(trans)
        
        output = []
                #find minimal unsatisfiable core by calling picomus
        if cmd is None:
            return (False, False, [], "")   
        
          
                
        #start a reader thread        
        subp = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=False)                                            
        readThread =  threading.Thread(target = subprocessReadThread, args=(subp.stdout,output))
        readThread.daemon = True
        readThread.start()
        
        
        #send header
        input = "p cnf "+str(p)+" "+str(n)+"\n"
        subp.stdin.write(input)                                      
        subp.stdin.writelines(init)               
           

        #Duplicating transition clauses for depth greater than 1         
        numOrigClauses = len(trans)  
        #the depth tells you how many time steps of trans to use
        #depth 0 just checks init with goals
        #p = 0
        
        
        for i in range(1,depth+1):
                    for clause in trans:
                        newClause = ""
                        for c in clause.split():
                            intC = int(c)
                            newClause= newClause + str(cmp(intC,0)*(abs(intC)+numProps*i)) +" "
                            #p = max(p, (abs(intC)+numProps*i))
                        newClause=newClause+"\n"
                        subp.stdin.write(newClause)
                    j = 0    
                    for line in conjuncts:
                        if "[]" in line and "<>" not in line:                      
                            numVarsInTrans = (len(mapping[line]))/(i+1)
                            mapping[line].extend(map(lambda x: x+numOrigClauses, mapping[line][-numVarsInTrans:]))
                            j = j + 1
                    #transClauses.extend(transClausesNew)                                   
                    #send this batch of transClauses
                    
        #create goal clauses
        dg = map(lambda x: ' '.join(map(lambda y: str(cmp(int(y),0)*(abs(int(y))+numProps*(depth))), x.split())) + '\n', goals)        
        #send goalClauses
        subp.stdin.writelines(dg)
        #send EOF
        subp.stdin.close()
        
                                
        nMinusG = n - len(goals)
        for line in conjuncts:
            if "<>" in line:
                mapping[line] = range(nMinusG+1,nMinusG+len(goals)+1)
                
        
        readThread.join()
        
    

        
                
                                                                                      
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__

        """#this is the BMC part: keep adding cnf clauses from the transitions until the spec becomes unsatisfiable
            if "UNSATISFIABLE" in output or depth >= maxDepth:
                    break
            depth = depth +1
            """
        if any(["WARNING: core extraction disabled" in s for s in output]):
            # never again
            print "************************************************"
            print "*** ERROR: picomus needs to be compiled with ***"
            print "*** trace support, or things will misbehave. ***"
            print "***                                          ***"
            print "*** Recompile with ./configure --trace       ***"
            print "************************************************"
            return []

        if any(["UNSATISFIABLE" in s for s in output]):
            print "Unsatisfiable core found at depth ", depth
        elif any(["SATISFIABLE" in s for s in output]):
            print "Satisfiable at depth ", depth
            return []
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
        cnfIndices = filter(lambda y: y!=0, map((lambda x: int(x.strip('v').strip())), filter(lambda z: re.match('^v', z), output)))
        
        #get corresponding LTL conjuncts
        guilty = cnfToConjuncts([idx for idx in cnfIndices if idx > ignoreBound], mapping, cnfMapping)
            
        return guilty
    
    
def unsatCoreCases(cmd, propList, topo, badInit, conjuncts,maxDepth,numRegions):
        numProps = len(propList)
        depth = numRegions
        
        #first try without topo and init, see if it is satisfiable
        ignoreDepth = 0    
        mapping, cnfMapping, init, trans, goals = conjunctsToCNF([badInit], conjuncts, propList)
           
        pool = Pool()
                      
            #print "STARTING PICO MAP"
            
        guiltyList = map(findGuiltyLTLConjunctsWrapper, itertools.izip(itertools.repeat(cmd),range(0,maxDepth + 1), itertools.repeat(numProps), itertools.repeat(init), itertools.repeat(trans), itertools.repeat(goals), itertools.repeat(mapping), itertools.repeat(cnfMapping), itertools.repeat(conjuncts),itertools.repeat(ignoreDepth)))
            #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
            
        pool.terminate()
            
        allGuilty = set([item for sublist in guiltyList for item in sublist])
            
        if all(guiltyList):
            print "unsat core found without topo and init"
            return trans, allGuilty
        else:
            ignoreDepth = len([g for g in allGuilty if g])
            depth += ignoreDepth
            
        #then try just topo and init and see if it is unsatisfiable. If so, return core.
        mapping,  cnfMapping, init, trans, goals = conjunctsToCNF([topo, badInit], [], propList)
       
                    
        guilty = findGuiltyLTLConjuncts(cmd,maxDepth,numProps,init,trans,goals,mapping,cnfMapping,[topo, badInit],0)
        
                #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
 
        if guilty:
            print "unsat core found with just topo and init"
            return trans, guilty
        
        #if the problem is in conjunction with the topo but not just topo, keep increasing the depth until something more than just topo is returned
        mapping,  cnfMapping, init, trans, goals = conjunctsToCNF([topo,badInit], conjuncts, propList)
        
        pool = Pool()
                      
            #print "STARTING PICO MAP"
            
        guiltyList = pool.map(findGuiltyLTLConjunctsWrapper, itertools.izip(itertools.repeat(cmd),range(maxDepth,maxDepth + 1), itertools.repeat(numProps), itertools.repeat(init), itertools.repeat(trans), itertools.repeat(goals), itertools.repeat(mapping),  itertools.repeat(cnfMapping), itertools.repeat([topo, badInit]+conjuncts), itertools.repeat(ignoreDepth)))
            #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
            
        pool.terminate()
        
        guilty = [item for sublist in guiltyList for item in sublist]        
        
        guiltyMinusGoal = [g for g in guilty if '<>' not in g]

        # don't use ignoreDepth for deadlock
        if len(goals) == 0:
           ignoreDepth = 0 
                        
        justTopo = set([topo, badInit]).issuperset(guiltyMinusGoal)
        depth = maxDepth + 1
        
        while justTopo:
            
            guilty = findGuiltyLTLConjuncts(cmd,depth,numProps,init,trans,goals,mapping,cnfMapping,[topo, badInit]+conjuncts, ignoreDepth)
            #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
            
            
            guiltyMinusGoal = [g for g in guilty if '<>' not in g]
            if not set([topo, badInit]).issuperset(set(guiltyMinusGoal)):
                justTopo = False
            else:
                depth+=1
            #get contributing conjuncts from CNF indices            
            #guilty = cnfToConjuncts(allIndices, mapping)
        
        print "unsat core found with all parts" 
        
        return trans, guilty
            
        
        
