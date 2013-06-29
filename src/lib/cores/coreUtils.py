
import math, re, sys, random, os, subprocess, time
from copy import copy, deepcopy
from logic import to_cnf
from multiprocessing import Pool
import threading
import itertools, functools
import logging
import random

USE_MULTIPROCESSING = False
props = {}

def function_star_wrapper(function, args):
    """ Work around lack of pool.starmap() and unpickleability of lambda functions.
        See http://stackoverflow.com/questions/5442910/python-multiprocessing-pool-map-for-multiple-arguments """
    return function(*args)

def runMap(function, inputs, multiarg=True):
    """ Wrapper for single- and multi-threaded versions of map, to make
        it easy to disable multiprocessing for debugging purposes.

        If `multiarg` is True, it will call `function(*x)` instead of `function(x)`
        for each `x` in `inputs`.
    """

    function_name = function.__name__

    logging.debug("[{}] Starting map ({}-threaded): {}".\
                  format(time.strftime("%H:%M:%S"),
                         "multi" if USE_MULTIPROCESSING else "single",
                         function_name))

    if multiarg:
        function = functools.partial(function_star_wrapper, function)

    if USE_MULTIPROCESSING:
        pool = Pool()
        outputs = pool.map(function, inputs, chunksize = 1)   
        pool.terminate()
    else:
        outputs = map(function, inputs)   

    logging.debug("[{}] Finished map: {}".format(time.strftime("%H:%M:%S"),
                                                 function_name))

    return outputs

def conjunctsToCNF(conjuncts, propList):
    #takes a list of LTL formulas and a list of propositions used in them,
    #and converts them into DIMACS CNF format replacing each proposition
    #with its index in the list. 
    #returns:
    #         mapping: a mapping from LTL formulas to CNF clause numbers
    #         cnfMapping: a mapping from LTL formulas to CNFs 
    #         cnfClauses: CNFS corresponding to initial and transition formulas
    #                     (represents one-step unrolling)
    #         transClauses: CNFS corresponding to transition formulas
    #                     (useful for further unrolling later)
    #         goalClauses: CNFS corresponding to goal formulas
    #                     (useful for checking goals at each time step)
    
    propListNext = map(lambda s: 'next_'+s, propList)
    
    global props
    props = {propList[x]:x+1 for x in range(0,len(propList))}
    propsNext = {propListNext[x]:len(propList)+x+1 for x in range(0,len(propListNext))}
    mapping = {conjuncts[x]:[] for x in range(0,len(conjuncts))}
    
    cnfClauses = []
    transClauses = []
    goalClauses = []
    n = 0 #counts number of clauses generated for mapping LTL to line numbers
    p = len(props)+len(propsNext)  
    
    
    allCnfs = runMap(lineToCnf, conjuncts, multiarg=False)   
    
    #associate original LTL conjuncts with CNF clauses
    cnfMapping = {line:cnf.split("&") for cnf, line in zip(allCnfs,conjuncts) if cnf}  
    for cnf, lineOld in zip(allCnfs,conjuncts):     
      if cnf: 
        allClauses = cnf.split("&");
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
                transClauses.append(clause.strip()+" 0\n")
                cnfClauses.append(clause.strip()+" 0\n")                                
            else:
                cnfClauses.append(clause.strip()+" 0\n")         
            
        if not "<>" in lineOld and "[]" in lineOld:
            #for non-goal (i.e. trans and init) formulas, extend mapping with line nos.
            #the guilty goal is always put last, so we don't need the clause nos.
            mapping[lineOld].extend(range(n+1,n+1+len(allClauses)))    
            n = n + len(allClauses)
                        
    return mapping, cnfMapping, cnfClauses, transClauses, goalClauses
    

def cnfToConjuncts(cnfIndices, mapping, cnfMapping):
    #takes a list of cnf line numbers and returns the corresponding LTL
    conjuncts = []

    cnfIndices = set(cnfIndices)
    #print sorted([a for k,b in mapping.iteritems() for a in b])

    for k in mapping.keys():
        if not set(mapping[k]).isdisjoint(cnfIndices):
            print "\tfrom conjunct ",k
            for i in range(len(mapping[k])):
                if mapping[k][i] in set(mapping[k]).intersection(cnfIndices):
                    print "\t\t", cnfMapping[k][i%len(cnfMapping[k])], ' at time step ', i/len(cnfMapping[k])
                           
            conjuncts.append(k)  
            cnfIndices -= set(mapping[k]).intersection(cnfIndices)
            #print k , (set(mapping[k]).intersection(cnfIndices))

    if cnfIndices:
        logging.warning("Unable to map all CNF indices to LTL. Leftovers are: {!r}".format(cnfIndices))
        raise IndexError(cnfIndices)

    return conjuncts


def lineToCnf(line):
        #converts a single LTL formula into CNF form 
        line = stripLTLLine(line)
        if line!='':
            line = re.sub('s\.','',line)
            line = re.sub('e\.','',line)   
            line = re.sub(r'(next\(\s*!)', r'(!next_', line)         
            line = re.sub(r'(next\(\s*)', r'(next_', line)
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
        
        
    
def stripLTLLine(line, useNext=False):
        #strip white text and LTL operators           
        line = re.sub('[\t\n]*','',line)    
        line = re.sub('\<\>','',line)  
        line = re.sub('\[\]','',line)  
        line = line.strip()
        #trailing &
        line = re.sub('&\s*$','',line)   
        if useNext:
            line = re.sub('s\.','next_s.',line)
            line = re.sub('e\.','next_e.',line)                     
        return line
        
def subprocessReadThread(fd, out):
            for line in fd:                                                                              
               out.append(line) 
                
def findGuiltyLTLConjuncts(cmd, depth, numProps, init, trans, goals, mapping,  cnfMapping, conjuncts, ignoreDepth): 
        #returns the ltl conjuncts returned as an unsat core when unrolling trans depth times from init and 
        #checking goal at final time step
        #note that init contains one-step unrolling of trans already
        
        mapping = deepcopy(mapping)
        #precompute p and n
        p = (depth+2)*(numProps)
        #the +2 is because init contains one trans already 
        #(so effectively there are depth+1 time steps and one final "next" time step)        
        
        n = (depth)*(len(trans)) + len(init) + len(goals)
        if ignoreDepth == 0:
            ignoreBound = 0
        else:
            ignoreBound = len(init) + (ignoreDepth)*len(trans)
            
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
        input = ["p cnf "+str(p)+" "+str(n)+"\n"]
        subp.stdin.write(input[0])                     
        subp.stdin.writelines(init)             
        input.extend(init)
        
        #Duplicating transition clauses for depth greater than 1         
        numOrigClauses = len(trans)  
        #the depth tells you how many time steps of trans to use
        #depth 0 just checks init with goals
       
        def incrementNumbers(m):
            """ Shift all numbers in a string by `numProps` away from zero.
                If zero, don't change. """
            num = int(m.group())
            return str(cmp(num,0)*(abs(num)+numProps))

        # mapping is LTL to cnf line #
        trans_mapping = {k:v for k,v in mapping.iteritems() if "[]" in k and "<>" not in k}
        current_trans = deepcopy(trans)
        for i in range(1, depth+1):
            for ltl_clause, cnf_line_nums in trans_mapping.iteritems():
                # Increment trans_mapping
                trans_mapping[ltl_clause] = [num+len(trans) for num in cnf_line_nums]
                # Update mapping
                mapping[ltl_clause].extend(trans_mapping[ltl_clause])

            for k, clause in enumerate(current_trans):
                new_clause = re.sub(r"((?:-)?\d+)", incrementNumbers, clause)
                current_trans[k] = new_clause
                #send this clause
                subp.stdin.write(new_clause)
                input.append(new_clause)
        
        #create goal clauses
        dg = map(lambda x: ' '.join(map(lambda y: str(cmp(int(y),0)*(abs(int(y))+numProps*(depth))), x.split())) + '\n', goals)        
        #send goalClauses
        subp.stdin.writelines(dg)
        input.extend(dg)
        #send EOF
        subp.stdin.close()
        
                                
        #update mapping with newly added clause line numbers
        nMinusG = n - len(goals)
        for line in conjuncts:
            if "<>" in line:
                mapping[line] = range(nMinusG+1,nMinusG+len(goals)+1)
                
        
        readThread.join()
        
    

        
                
                                                                                      
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__

        #get indices of contributing clauses
        cnfIndices = filter(lambda y: y!=0, map((lambda x: int(x.strip('v').strip())), filter(lambda z: re.match('^v', z), output)))
        
#        
#        #Write input to file (mainly for debugging purposes)
#        satFileName = "input"+str(random.randint(0,1000))+".sat"
#        inputFile = open(satFileName,'w')
#        inputFile.write("\n".join(input))
#        logging.debug("wrote {}".format(satFileName))
#        inputFile.close()
            
        if any(("WARNING: core extraction disabled" in s for s in output)):
            # never again
            logging.error("************************************************")
            logging.error("*** ERROR: picomus needs to be compiled with ***")
            logging.error("*** trace support, or things will misbehave. ***")
            logging.error("***                                          ***")
            logging.error("*** Recompile with ./configure --trace       ***")
            logging.error("************************************************")
            return []

        if any(("UNSATISFIABLE" in s for s in output)):
            logging.info("Unsatisfiable core found at depth {}".format(depth))
        elif any(("SATISFIABLE" in s for s in output)):
            logging.info("Satisfiable at depth {}".format(depth))
            return []
        else:
            logging.error("Picosat error: {!r}".format(output))
        
        #Write output to file (mainly for debugging purposes)

        satFileName = "debug"+str(random.randint(0,1000))+".sat"
        with open(satFileName,'w') as outputFile:
            outputFile.writelines(makeHumanReadableCNFFromIndices(cnfIndices, input, ignoreBound))
        logging.debug("wrote {}".format(satFileName))

        #get corresponding LTL conjuncts
        try:
            guilty = cnfToConjuncts([idx for idx in cnfIndices if idx > ignoreBound], mapping, cnfMapping)
        except IndexError as e:
            unmappedIndices = e.args[0]
            logging.warning("The unmapped conjuncts are as follows:\n" + \
                            ''.join(makeHumanReadableCNFFromIndices(unmappedIndices, input, ignoreBound)))
            return []
            
        return guilty

def makeHumanReadableCNFFromIndices(indices, input_data, ignoreBound):
    def cnfVar2LTLVar(m):
        global props
        negated = (m.group("sign") == "-")
        num = int(m.group("num"))

        if num == 0:
            return ""

        time_step = int(math.floor((num - 1)/len(props)))
        var_num = (num - len(props)*time_step)

        for k,v in props.iteritems():
            if v == var_num:
                var_name = k
                break

        name = "{}@{}".format(var_name, time_step)
        if negated:
            name = "~"+name
        return name
        
    output = []
    for input_line_num in indices: 
        cnf_clause = input_data[input_line_num]
        cnf_clause = re.sub(r"(?P<sign>-)?(?P<num>\d+)", cnfVar2LTLVar, cnf_clause)
        if input_line_num <= ignoreBound:
            cnf_clause = cnf_clause.rstrip() + " (ignored)\n"
        output.append(cnf_clause)

    return output
 
def unsatCoreCases(cmd, propList, topo, badInit, conjuncts, maxDepth, numRegions):
     #returns the minimal unsatisfiable core (LTL formulas) given
     #        cmd: picosat command
     #        propList: list of proposition names used
     #        topo: LTL formula describing topology
     #        badInit: formula describing bad initial states
     #        conjuncts: remaining LTL formulas highlighted by preliminary analysis
     #        maxDepth: determines how many time steps we unroll 
     #        numRegions: used to determine minimum depth to prevent false alarms (every depth between numRegions+1 and maxDepth is checked)
       
        numProps = len(propList)
        #initial depth is set to the number of regions. This ensures that we unroll at least as 
        #far as needed to physically get to the goal
        depth = numRegions
        
        #first try without topo and init, see if it is satisfiable
        ignoreDepth = 0    
        mapping, cnfMapping, init, trans, goals = conjunctsToCNF([badInit]+conjuncts, propList)
        
        logging.info("Trying to find core without topo or init") 

        guiltyList = runMap(findGuiltyLTLConjuncts, itertools.izip(itertools.repeat(cmd),
                                                                          range(1, maxDepth + 1),
                                                                          itertools.repeat(numProps),
                                                                          itertools.repeat(init),
                                                                          itertools.repeat(trans), 
                                                                          itertools.repeat(goals),
                                                                          itertools.repeat(mapping),
                                                                          itertools.repeat(cnfMapping),
                                                                          itertools.repeat(conjuncts),
                                                                          itertools.repeat(ignoreDepth)))

        #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            
        allGuilty = set([item for sublist in guiltyList for item in sublist])
            
        if all(guiltyList):
            logging.info("Unsat core found without topo or init")
            return trans, allGuilty
        else:
            ignoreDepth = len([g for g in guiltyList if g])
            depth += ignoreDepth
        
        logging.info("ignore depth {}".format(ignoreDepth))
            
        #then try just topo and init and see if it is unsatisfiable. If so, return core.
        logging.info("Trying to find core with just topo and init") 
        mapping,  cnfMapping, init, trans, goals = conjunctsToCNF([topo, badInit], propList)
       
                    
        guilty = findGuiltyLTLConjuncts(cmd,maxDepth,numProps,init,trans,goals,mapping,cnfMapping,[topo, badInit],0)
        
                #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
 
        if guilty:
            logging.info("Unsat core found with just topo and init")
            return trans, guilty
        
        #if the problem is in conjunction with the topo but not just topo, keep increasing the depth until something more than just topo is returned
        mapping,  cnfMapping, init, trans, goals = conjunctsToCNF([topo,badInit] + conjuncts, propList)
        
        logging.info("Trying to find core with everything")
        

        guiltyList = runMap(findGuiltyLTLConjuncts, itertools.izip(itertools.repeat(cmd),
                                                                          range(maxDepth, maxDepth + 1),
                                                                          itertools.repeat(numProps),
                                                                          itertools.repeat(init),
                                                                          itertools.repeat(trans),
                                                                          itertools.repeat(goals),
                                                                          itertools.repeat(mapping),
                                                                          itertools.repeat(cnfMapping),
                                                                          itertools.repeat([topo, badInit] + conjuncts),
                                                                          itertools.repeat(ignoreDepth)))

        #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
        
        guilty = [item for sublist in guiltyList for item in sublist]        
        
        guiltyMinusGoal = [g for g in guilty if '<>' not in g]

        # don't use ignoreDepth for deadlock
        if len(goals) == 0:
           ignoreDepth = 0 
                        
        justTopo = set([topo, badInit]).issuperset(guiltyMinusGoal)
        depth = maxDepth + 1
        
        #while justTopo and depth < maxDepth:
            
            #guilty = findGuiltyLTLConjuncts(cmd,depth,numProps,init,trans,goals,mapping,cnfMapping,[topo, badInit]+conjuncts, ignoreDepth)
            ##allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            ##print "ENDING PICO MAP"
            
            
            #guiltyMinusGoal = [g for g in guilty if '<>' not in g]
            #if not set([topo, badInit]).issuperset(set(guiltyMinusGoal)):
                #justTopo = False
            #else:
                #depth+=1
            ##get contributing conjuncts from CNF indices            
            ##guilty = cnfToConjuncts(allIndices, mapping)
        
        logging.info("Unsat core found with all parts")
        
        return trans, guilty
    
def stateToLTL(state, useEnv=1, useSys=1, use_next=False):
        def decorate_prop(prop, polarity):
            if int(polarity) == 0:
                prop = "!"+prop
            if use_next:
                prop = "next({})".format(prop)
            return prop
            
       
        sys_state = " & ".join([decorate_prop("s."+p, v) for p,v in state.inputs.iteritems()])
        env_state = " & ".join([decorate_prop("e."+p, v) for p,v in state.outputs.iteritems()])
                
        if useEnv:
            if useSys:
                return env_state + " & " + sys_state
            else:
                return env_state
        elif useSys:
            return sys_state
        else:
            return ""
            
        
        
