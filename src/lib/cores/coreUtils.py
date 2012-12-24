
import math, re, sys, random, os, subprocess, time
from logic import to_cnf

def fileToconjuncts(ltlFile, component):
    
    p = 0
    n = 0
    props = {propList[x]:x+1 for x in range(0,len(propList))}
    mapping = {conjuncts[x]:[] for x in range(0,len(conjuncts))}
    
    cnfClauses = []
    n = 0
    p = len(props)
        
    for line in conjuncts:
        lineOld = line
        line = re.sub('!', '~', line)
        #line = re.sub('&\s*\n', '', line)
        line = re.sub('[\s]+', ' ', line)
        line = line.strip()       
        cnf = str(to_cnf(line))
        allClauses = cnf.split("&");
        #associate original conjuncts with CNF clauses
        mapping[lineOld] = range(n+1,n+1+len(allClauses))
        n = n + len(allClauses)
        for clause in allClauses:    
            clause = re.sub('[()]', '', clause)   
            clause = re.sub('[|]', '', clause)           
            clause = re.sub('~', '-', clause)    
            for k in props.keys():
                clause = re.sub(k,str(props[k]), clause)
            cnfClauses.append(clause.strip()+" 0\n")
            
    #write CNFs to file        
    open('out.cnf', 'w').close()
    output = open("out.cnf", 'a')
    output.write("p cnf "+str(p)+" "+str(n)+"\n")
    output.writelines(cnfClauses)
    output.close()
    
    return mapping

def conjunctsToCNF(conjuncts, propList, outFilename):
    
    p = 0
    n = 0
    propListNext = map(lambda s: 'next_'+s, propList)
    
    props = {propList[x]:x+1 for x in range(0,len(propList))}
    propsNext = {propListNext[x]:len(propList)+x+1 for x in range(0,len(propListNext))}
    mapping = {conjuncts[x]:[] for x in range(0,len(conjuncts))}
    
    cnfClauses = []
    n = 0
    p = len(props)+len(propsNext)
    
    
        
    for line in conjuncts:
        lineOld = line
        line = re.sub('[\t\n]*','',line)            
        line = re.sub('s\.','',line)
        line = re.sub('e\.','',line)   
        line = re.sub(r'(next\()', r'(next_', line) 
        line = re.sub('\[\]','',line)  
        line = line.strip()
        #training &
        line = line[:-1]
        if line=='':
            continue
        line = re.sub('!', '~', line)
        #line = re.sub('&\s*\n', '', line)
        line = re.sub('[\s]+', ' ', line)        
        line = re.sub('\<-\>', '<=>', line)
        line = re.sub('->', '>>', line)
        line = line.strip()   
        cnf = str(to_cnf(line))
        allClauses = cnf.split("&");
        #associate original conjuncts with CNF clauses
        mapping[lineOld] = mapping[lineOld] + (range(n+1,n+1+len(allClauses)))
        n = n + len(allClauses)
        for clause in allClauses:    
            clause = re.sub('[()]', '', clause)   
            clause = re.sub('[|]', '', clause)           
            clause = re.sub('~', '-', clause)    
            for k in propsNext.keys():
                clause = re.sub(k,str(propsNext[k]), clause)
            for k in props.keys():
                clause = re.sub(k,str(props[k]), clause)            
            cnfClauses.append(clause.strip()+" 0\n")
            
    #write CNFs to file        
    open(outFilename, 'w').close()
    output = open(outFilename, 'a')
    output.write("p cnf "+str(p)+" "+str(n)+"\n")
    output.writelines(cnfClauses)
    output.close()
    return mapping


def cnfToConjuncts(cnfIndices, mapping):
    conjuncts = []
    i = 0
    for k in mapping.keys():
        i = i + 1
        if not set(mapping[k]).isdisjoint(cnfIndices):
            conjuncts.append(k)
    return conjuncts
#for line in input:
#    temp = open("temp.exp", 'w')
#    line = re.sub('&\s*\n', '', line)
#    line = re.sub('[\s]+', ' ', line)
#    line = line.strip()   
#    temp.write(line+'\n')
#    temp.close()
#    
#    cmd = "a.exe < temp.exp > temp.cnf"
#    os.system(cmd)
#    
#    
#    
#    cnfs = open("temp.cnf", 'r')
#    output = open("out.cnf", 'a')
#    
#    newp = 0
#    newn = 0
#    for clause in cnfs:
#     #clauses = re.findall(r'\d+', cnfs)
#         if re.match('p', clause) is None:
#            increment(clause,n)
#            output.write(clause)
#         else:
#            nums = re.split('\W+', clause)
#            newp = int(nums[2])
#            newn = int(nums[3])
#    
#    cnfs.close()
#    output.close()
#     
#    p = p + newp
#    n = n + newn
#    
#input.close()
#     
      



#last_number = re.compile(r'\d+(?=[^\d]*$)')

#def increment(string, k):
#  def increment_number(match):
#    num_str = match.group(1)
#      str( int(num_str) + k).zfill( len(num_str) )

#return last_number.sub(increment_number, string)


     
      



#last_number = re.compile(r'\d+(?=[^\d]*$)')

#def increment(string, k):
#  def increment_number(match):
#    num_str = match.group(1)
#      str( int(num_str) + k).zfill( len(num_str) )

#return last_number.sub(increment_number, string)

