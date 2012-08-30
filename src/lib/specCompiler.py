import os, sys
import re
import time, copy
import math
import subprocess

# Add SLURP to path for import
# Climb the tree to find out where we are

p = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(p, "..", "etc", "SLURP"))

from ltlbroom.specgeneration import SpecGenerator

sys.path.append("lib")

import project
import parseLP
from createJTLVinput import createLTLfile, createSMVfile
from parseEnglishToLTL import bitEncoding, replaceRegionName

class SpecCompiler(object):
    def __init__(self, spec_filename):
        self.proj = project.Project()
        self.proj.loadProject(spec_filename)

        # Check to make sure this project is complete
        if self.proj.rfi is None:
            print "ERROR: Please define regions before compiling."
            return
    
        if self.proj.specText.strip() == "":
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
        
    def _writeSMVFile(self):
        numRegions = len(self.parser.proj.rfi.regions)
        sensorList = self.proj.enabled_sensors
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs + self.proj.internal_props

        createSMVfile(self.proj.getFilenamePrefix(), numRegions, sensorList, robotPropList)

    def _writeLTLFile(self):
        #regionList = [r.name for r in self.parser.proj.rfi.regions]
        regionList = [r.name for r in self.proj.rfi.regions]
        sensorList = copy.deepcopy(self.proj.enabled_sensors)
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs
        
        # Allow the option of not running decomposition
        if self.decomposedSpecText is not None:
            text = self.decomposedSpecText
        else:
            text = self.proj.specText

        # Make a new specgenerator and have it process the text
        specGen = SpecGenerator()
        LTLspec_env, LTLspec_sys, self.proj.internal_props, responses, traceback = \
            specGen.generate(text, sensorList, regionList, robotPropList)

        for ln, response in enumerate(responses):
            if not response:
                print "WARNING: Could not parse the sentence in line {0}".format(ln)

        # Abort compilation if there were any errors
        if not all(responses):
            return None
    
        # Add in the internal memory propositions, so they go into the SMV and spec files
        for p in self.proj.internal_props:
            if p not in robotPropList:
                robotPropList.append(p)

        # Conjoin all the spec chunks
        LTLspec_env = '\t\t' + ' & \n\t\t'.join(LTLspec_env)
        LTLspec_sys = '\t\t' + ' & \n\t\t'.join(LTLspec_sys)

        # substitute decomposed region names
        for r in self.proj.rfi.regions:
            if not (r.isObstacle or r.name.lower() == "boundary"):
                LTLspec_env = re.sub('\\bs\.' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_env)
                LTLspec_sys = re.sub('\\bs\.' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_sys)

        # Prepend "e." or "s." to propositions for JTLV
        for i, sensor in enumerate(sensorList):
            text = re.sub("\\b"+sensor+"\\b", "e." + sensor, text)
            sensorList[i] = "e." + sensorList[i]

        for i, prop in enumerate(robotPropList):
            text = re.sub("\\b"+prop+"\\b", "s." + prop, text)
            robotPropList[i] = "s." + robotPropList[i]

        regionList = [x.name for x in self.parser.proj.rfi.regions]

        # Define the number of bits needed to encode the regions
        numBits = int(math.ceil(math.log(len(regionList),2)))

        # creating the region bit encoding
        bitEncode = bitEncoding(len(regionList),numBits)
        currBitEnc = bitEncode['current']
        nextBitEnc = bitEncode['next']

        # switch to bit encodings for regions
        LTLspec_env = replaceRegionName(LTLspec_env, bitEncode, regionList)
        LTLspec_sys = replaceRegionName(LTLspec_sys, bitEncode, regionList)

        adjData = self.parser.proj.rfi.transitions

        createLTLfile(self.proj.getFilenamePrefix(), sensorList, robotPropList, adjData, LTLspec_env, LTLspec_sys)

        return traceback
        
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
        nonTrivial = False

        output = ""
        to_highlight = []
        for dline in subp.stdout:
            output += dline
            if "Specification is realizable." in dline:   
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
                to_highlight.append(("env", "trans"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("env", "goals", int(l)))
            elif "Environment initial condition inconsistent with transition relation" in dline:
                to_highlight.append(("env", "init"))
                to_highlight.append(("env", "trans"))
           
        
            # System unrealizability
            elif "System is unrealizable because the environment can force a safety violation" in dline:
                to_highlight.append(("sys", "trans"))
            elif "System highlighted goal(s) unrealizable" in dline:
                to_highlight.append(("sys", "trans"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("sys", "goals", int(l)))
            
            # Environment unrealizability
            elif "Environment is unrealizable because the system can force a safety violation" in dline:
                to_highlight.append(("env", "trans"))
            elif "Environment highlighted goal(s) unrealizable" in dline:
                to_highlight.append(("env", "trans"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("env", "goals", int(l)))

        # check for trivial initial-state automaton with no transitions
        if realizable:
            proj_copy = deepcopy(self.proj)
            proj_copy.rfi = self.proj.parser.rfi
            proj_copy.sensor_handler = None
            proj_copy.actuator_handler = None
            proj_copy.h_instance = None

            aut = fsa.Automaton(proj_copy)

            aut.loadFile(self.proj.getFilenamePrefix()+".aut", self.proj.enabled_sensors, self.proj.enabled_actuators, self.proj.all_customs)
            
            nonTrivial = any([s.transitions != [] for s in aut.states])

        subp.stdout.close()

        return (realizable, nonTrivial, to_highlight, output)

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
        self._decompose()
        tb = self._writeLTLFile()
        self._writeSMVFile()

        if tb is None:
            print "ERROR: Compilation aborted"
            return 

        #self._checkForEmptyGaits()
        self._synthesize(with_safety_aut)

