import os, sys
import re
import time
import subprocess
import numpy
import itertools

from multiprocessing import Pool

sys.path.append("lib")

import project
import parseLP
from createJTLVinput import createLTLfile, createSMVfile
from parseEnglishToLTL import writeSpec
import fsa
from copy import deepcopy
from coreUtils import *



class SpecCompiler(object):
    def __init__(self, spec_filename):
        self.proj = project.Project()
        self.proj.loadProject(spec_filename)

        # Check to make sure this project is complete
        if self.proj.rfi is None:
            print "ERROR: Please define regions before compiling."
            return
    
        # Remove comments
        self.specText = re.sub(r"#.*$", "", self.proj.specText, flags=re.MULTILINE)

        if self.specText.strip() == "":
            print "ERROR: Please write a specification before compiling."
            return

        self.decomposedSpecText = None

    def _decompose(self):
        self.parser = parseLP.parseLP()
        self.parser.main(self.proj.getFilenamePrefix() + ".spec")

        # Remove all references to any obstacle regions at this point
        for r in self.proj.rfi.regions:
            if r.isObstacle:
                # Delete corresponding decomposed regions
                for sub_r in self.parser.proj.regionMapping[r.name]:
                    del self.parser.proj.rfi.regions[self.parser.proj.rfi.indexOfRegionWithName(sub_r)]

                    # Remove decomposed region from any overlapping mappings
                    for k,v in self.parser.proj.regionMapping.iteritems(): 
                        if k == r.name: continue
                        if sub_r in v:
                            v.remove(sub_r)

                # Remove mapping for the obstacle region
                del self.parser.proj.regionMapping[r.name]

        #self.proj.rfi.regions = filter(lambda r: not (r.isObstacle or r.name == "boundary"), self.proj.rfi.regions)
                    
        # save the regions into new region file
        filename = self.proj.getFilenamePrefix() + '_decomposed.regions'
        self.parser.proj.rfi.recalcAdjacency()
        self.parser.proj.rfi.writeFile(filename)

        self.proj.regionMapping = self.parser.proj.regionMapping
        self.proj.writeSpecFile()
        
        # substitute the regions name in specs
        text = self.specText
        for m in re.finditer(r'near (?P<rA>\w+)', text):
            text=re.sub(r'near (?P<rA>\w+)', "("+' or '.join(self.parser.proj.regionMapping['near$'+m.group('rA')+'$'+str(50)])+")", text)
        for m in re.finditer(r'within (?P<dist>\d+) (from|of) (?P<rA>\w+)', text):
            text=re.sub(r'within ' + m.group('dist')+' (from|of) '+ m.group('rA'), "("+' or '.join(self.parser.proj.regionMapping['near$'+m.group('rA')+'$'+m.group('dist')])+")", text)
        for m in re.finditer(r'between (?P<rA>\w+) and (?P<rB>\w+)', text):
            text=re.sub(r'between ' + m.group('rA')+' and '+ m.group('rB'),"("+' or '.join(self.parser.proj.regionMapping['between$'+m.group('rA')+'$and$'+m.group('rB')+"$"])+")", text)
        for r in self.proj.rfi.regions:
            if not (r.isObstacle or r.name.lower() == "boundary"):
                text=re.sub('\\b' + r.name + '\\b', "("+' or '.join(self.parser.proj.regionMapping[r.name])+")", text)

        self.decomposedSpecText = text

    def _writeSMVFile(self):
        numRegions = len(self.parser.proj.rfi.regions)
        sensorList = self.proj.enabled_sensors
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs

        createSMVfile(self.proj.getFilenamePrefix(), numRegions, sensorList, robotPropList)

    def _writeLTLFile(self):
        regionList = [r.name for r in self.parser.proj.rfi.regions]
        sensorList = self.proj.enabled_sensors
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs
        
        # Allow the option of not running decomposition
        if self.decomposedSpecText is not None:
            text = self.decomposedSpecText
        else:
            text = self.specText

        self.spec, self.traceback, failed, self.LTL2LineNo = writeSpec(text, sensorList, regionList, robotPropList)

        # Abort compilation if there were any errors
        if failed:
            return None

        adjData = self.parser.proj.rfi.transitions

        createLTLfile(self.proj.getFilenamePrefix(), sensorList, robotPropList, adjData, self.spec)
        
        regNum = len(regionList)
        regList = map(lambda i: "bit"+str(i), range(0,int(numpy.ceil(numpy.log2(regNum)))))
        self.propList = sensorList + robotPropList + regList;

        return (self.propList, self.spec, self.traceback, self.LTL2LineNo)
        
    def _checkForEmptyGaits(self):
        from simulator.ode.ckbot import CKBotLib

        # Initialize gait library
        self.library = CKBotLib.CKBotLib()

        err = 0
        libs = self.library
        libs.readLibe()
		# Check that each individual trait has a corresponding config-gait pair
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs
        for act in robotPropList:
            act = act.strip("u's.")
            if act[0] == "T":
                act = act.strip("T_")
                #print act
                words = act.split("_and_")
                #print words
                config = libs.findGait(words)
                #print config
                if type(config) == type(None):
                    err_message = "WARNING: No config-gait pair for actuator T_" + act + "\n"
                    print err_message
                    err = 1

    def _getGROneCommand(self, module):
        # Check that GROneMain, etc. is compiled
        if not os.path.exists(os.path.join(self.proj.ltlmop_root,"etc","jtlv","GROne","GROneMain.class")):
            print "Please compile the synthesis Java code first.  For instructions, see etc/jtlv/JTLV_INSTRUCTIONS."
            # TODO: automatically compile for the user
            return None

        # Windows uses a different delimiter for the java classpath
        if os.name == "nt":
            delim = ";"
        else:
            delim = ":"

        classpath = delim.join([os.path.join(self.proj.ltlmop_root, "etc", "jtlv", "jtlv-prompt1.4.0.jar"), os.path.join(self.proj.ltlmop_root, "etc", "jtlv", "GROne")])

        cmd = ["java", "-ea", "-Xmx512m", "-cp", classpath, module, self.proj.getFilenamePrefix() + ".smv", self.proj.getFilenamePrefix() + ".ltl"]

        return cmd

    def _analyze(self):
        cmd = self._getGROneCommand("GROneDebug")
        if cmd is None:
            return (False, False, [], "")

        subp = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=False)

        # TODO: Make this output live
        while subp.poll():
            time.sleep(0.1)

        realizable = False    
        unsat = False
        nonTrivial = False
        

        output = ""
        to_highlight = []
        for dline in subp.stdout:
            output += dline
            if "Specification is realizable" in dline:   
                realizable = True            
            
            ### Highlight sentences corresponding to identified errors ###

            # System unsatisfiability
            elif "System initial condition is unsatisfiable." in dline:
                to_highlight.append(("sys", "init"))
            elif "System transition relation is unsatisfiable." in dline:
                to_highlight.append(("sys", "trans"))
            elif "System highlighted goal(s) unsatisfiable" in dline:
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("sys", "goals", int(l)))
            elif "System highlighted goal(s) inconsistent with transition relation" in dline:
                to_highlight.append(("sys", "trans"))
                to_highlight.append(("sys", "init"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("sys", "goals", int(l)))
            elif "System initial condition inconsistent with transition relation" in dline:
                to_highlight.append(("sys", "init"))
                to_highlight.append(("sys", "trans"))
           
            # Environment unsatisfiability
            elif "Environment initial condition is unsatisfiable." in dline:
                to_highlight.append(("env", "init"))
            elif "Environment transition relation is unsatisfiable." in dline:
                to_highlight.append(("env", "trans"))
            elif "Environment highlighted goal(s) unsatisfiable" in dline:
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("env", "goals", int(l)))
            elif "Environment highlighted goal(s) inconsistent with transition relation" in dline:
                to_highlight.append(("env", "init"))
                to_highlight.append(("env", "trans"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("env", "goals", int(l)))
            elif "Environment initial condition inconsistent with transition relation" in dline:
                to_highlight.append(("env", "init"))
                to_highlight.append(("env", "trans"))
           
        
            # System unrealizability
            elif "System is unrealizable because the environment can force a safety violation" in dline:
                to_highlight.append(("sys", "trans"))
                to_highlight.append(("sys", "init"))
            elif "System highlighted goal(s) unrealizable" in dline:
                to_highlight.append(("sys", "trans"))
                to_highlight.append(("sys", "init"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("sys", "goals", int(l)))
            
            # Environment unrealizability
            elif "Environment is unrealizable because the system can force a safety violation" in dline:
                to_highlight.append(("env", "trans"))
            elif "Environment highlighted goal(s) unrealizable" in dline:
                to_highlight.append(("env", "trans"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("env", "goals", int(l)))
                    
            if "unsatisfiable" in dline or "inconsistent" in dline :
                unsat = True
            
                
        # check for trivial initial-state automaton with no transitions
        if realizable:           
            proj_copy = deepcopy(self.proj)
            proj_copy.rfi = self.parser.proj.rfi
            proj_copy.sensor_handler = None
            proj_copy.actuator_handler = None
            proj_copy.h_instance = None
    
            aut = fsa.Automaton(proj_copy)
    
            aut.loadFile(self.proj.getFilenamePrefix()+".aut", self.proj.enabled_sensors, self.proj.enabled_actuators, self.proj.all_customs)        
            
            nonTrivial = any([s.transitions != [] for s in aut.states])

        subp.stdout.close()
        
        
        
        return (realizable, unsat, nonTrivial, to_highlight, output)
    
    
    def _coreFinding(self, to_highlight, unsat, badInit):
        #find number of states in automaton/counter for unsat/unreal core max unrolling depth ("recurrence diameter")
        proj_copy = deepcopy(self.proj)
        proj_copy.rfi = self.parser.proj.rfi
        proj_copy.sensor_handler = None
        proj_copy.actuator_handler = None
        proj_copy.h_instance = None
    
        aut = fsa.Automaton(proj_copy)
        aut.loadFile(self.proj.getFilenamePrefix()+".aut", self.proj.enabled_sensors, self.proj.enabled_actuators, self.proj.all_customs)        
        numStates = len(aut.states)
        
        numRegions = len(self.parser.proj.rfi.regions)
#        robotPropList = self.proj.enabled_actuators + self.proj.all_customs
#        regList = map(lambda i: "bit"+str(i), range(0,int(numpy.ceil(numpy.log2(len(regionList))))))
        
#        numStates = 2**len(regList + robotPropList)
        
        
        #get conjuncts to be minimized
        topo, conjuncts = self.getGuiltyConjuncts(to_highlight, badInit)
        
        if unsat:
            guilty = self.findCoresUnsat(topo,badInit,conjuncts,numStates,numRegions)#returns LTL  
        else:
            guilty = self.findCoresUnreal(topo,badInit,conjuncts,numStates,numRegions)#returns LTL   
        return guilty
        
        
        
    
    def findCoresUnsat(self,topo, badInit, conjuncts,maxDepth,numRegions):
        depth = numRegions
        cmd = self._getPicosatCommand() 
        numProps = len(self.propList)
        if not conjuncts and badInit == "":
            return []
        #first try without topo and init, see if it is satisfiable
        mapping, init, self.trans, goals = conjunctsToCNF([badInit], conjuncts, self.propList)
            
           
        pool = Pool()
                      
            #print "STARTING PICO MAP"
            
        guiltyList = pool.map(findGuiltyClausesWrapper, itertools.izip(itertools.repeat(cmd),range(0,maxDepth + 1), itertools.repeat(numProps), itertools.repeat(init), itertools.repeat(self.trans), itertools.repeat(goals), itertools.repeat(mapping), itertools.repeat(conjuncts)))
            #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
            
        pool.terminate()
            
        allGuilty = set([item for sublist in guiltyList for item in sublist])
            
        if all(guiltyList):
            return allGuilty
        else:
            depth += len([g for g in allGuilty if g])
        
        #then try just topo and init and see if it is unsatisfiable. If so, return core.
        mapping, init, self.trans, goals = conjunctsToCNF([topo, badInit], [], self.propList)
       
                    
        guilty = findGuiltyClauses(cmd,maxDepth,numProps,init,self.trans,goals,mapping,[topo, badInit])
        
                #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
            
            
            
        if guilty:
            return guilty
        
        #if the problem is in conjunction with the topo but not just topo, keep increasing the depth until something more than just topo is returned
        mapping, init, self.trans, goals = conjunctsToCNF([topo,badInit], conjuncts, self.propList)
        
        pool = Pool()
                      
            #print "STARTING PICO MAP"
            
        guiltyList = pool.map(findGuiltyClausesWrapper, itertools.izip(itertools.repeat(cmd),range(0,maxDepth + 1), itertools.repeat(numProps), itertools.repeat(init), itertools.repeat(self.trans), itertools.repeat(goals), itertools.repeat(mapping), itertools.repeat([topo, badInit]+conjuncts)))
            #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
            
        pool.terminate()
        
        guilty = [item for sublist in guiltyList for item in sublist]        
        
        guiltyMinusGoal = [g for g in guilty if '<>' not in g]
                        
        justTopo = set([topo, badInit]).issuperset(guiltyMinusGoal)
        depth = maxDepth + 1
        
        while justTopo:
            
            guilty = findGuiltyClauses(cmd,depth,numProps,init,self.trans,goals,mapping,[topo, badInit]+conjuncts)
            #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
            
            
            guiltyMinusGoal = [g for g in guilty if '<>' not in g]
            if not set([topo, badInit]).issuperset(set(guiltyMinusGoal)):
                justTopo = False
            else:
                depth+=1
            #get contributing conjuncts from CNF indices            
            #guilty = cnfToConjuncts(allIndices, mapping)
        
                    
        return guilty
    
          
        
                
    
        
    def findCoresUnreal(self,topo,badInit,conjuncts,maxDepth,numRegions):
        #get conjuncts to be minimized
        return []#self.findCoresUnsat(topo,badInit,conjuncts,maxDepth,numRegions)
        
        
    def _getPicosatCommand(self):
        # Check that GROneMain, etc. is compiled
        if not os.path.exists(os.path.join(self.proj.ltlmop_root,"lib","cores","picosat-951")):
            print "Where is your sat solver? We use Picomus."
            # TODO: automatically compile for the user
            return None

        classpath = os.path.join(self.proj.ltlmop_root, "lib","cores","picosat-951")

        #cmd = os.path.join(classpath,"picomus.exe ")+ self.proj.getFilenamePrefix() + ".cnf"
        cmd = os.path.join(classpath,"picomus.exe ")
        

        return cmd
    
    def getGuiltyConjuncts(self, to_highlight, badInit):  
        #inverse dictionary for goal lookups
        #ivd=dict([(v,k) for (k,v) in self.LTL2LineNo.items()])
        
        topo=self.spec['Topo'].replace('\n','')
        topo = topo.replace('\t','')
        
        if badInit != "":
            conjuncts = [badInit]
        else:
            conjuncts = []
                
        for h_item in to_highlight:
            tb_key = h_item[0].title() + h_item[1].title()

            newCs = []
            if h_item[1] == "goals":
                #special treatment for goals: (1) we already know which one to highlight, and (2) we need to check both tenses
                #TODO: separate out the check for present and future tense -- what if you have to toggle but can still do so infinitely often?
                #newCs = ivd[self.traceback[tb_key][h_item[2]]].split('\n')                 
                goals = self.spec[tb_key].split('\n')
                newCs = [goals[h_item[2]]]
                newCsOld = newCs
                """for p in self.propList:
                    old = ''+str(p)
                    new = 'next('+str(p)+')'
                    newCs = map(lambda s: s.replace(old,new), newCs) 
                """                           
                #newCs.extend(newCsOld)
            elif h_item[1] == "trans" or not badInit:
                #newCs = self.spec[tb_key].split('\n')
                newCs =  self.spec[tb_key].replace("\t", "\n").split("\n")
                #newCs = [k.split('\n') for k,v in self.LTL2LineNo.iteritems() if v in self.traceback[tb_key]]
                #newCs = [item for sublist in newCs for item in sublist]                
            """for clause in newCs:
                #need to mark trans lines because they do not always contain [] because of line breaks
                if h_item[1] == "trans":
                    isTrans[clause] = 1                    
                else:
                    isTrans[clause] = 0  
            """
            conjuncts.extend(newCs)
        
        #filter out props that are actually used
        self.propList = [p for p in self.propList if [c for c in conjuncts if p in c] or p in topo]
            
            
            
    
        return topo, conjuncts

    def _synthesize(self, with_safety_aut=False):
        cmd = self._getGROneCommand("GROneMain")
        if cmd is None:
            return (False, False, "")

        if with_safety_aut:    # Generally used for Mopsy
            cmd.append("--safety")

        if self.proj.compile_options["fastslow"]:
            cmd.append("--fastslow")

        subp = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=False)
        
        # TODO: Make this output live
        while subp.poll():
            time.sleep(0.1)

        realizable = False
        realizableFS = False

        output = ""
        for line in subp.stdout:
            output += line
            if "Specification is realizable" in line:
                realizable = True
            if "Specification is realizable with slow and fast actions" in line:
                realizableFS = True
               
        subp.stdout.close()

        return (realizable, realizableFS, output)

    def compile(self, with_safety_aut=False):
        print "--> Decomposing..."
        self._decompose()
        print "--> Writing LTL file..."
        tb = self._writeLTLFile()
        print "--> Writing SMV file..."
        self._writeSMVFile()

        if tb is None:
            print "ERROR: Compilation aborted"
            return 

        #self._checkForEmptyGaits()
        print "--> Synthesizing..."
        return self._synthesize(with_safety_aut)

