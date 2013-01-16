import os, sys
import re
import time, copy
import math
import subprocess

sys.path.append("lib")

import project
import regions
import parseLP
import fsa
from createJTLVinput import createLTLfile, createSMVfile
from parseEnglishToLTL import bitEncoding, replaceRegionName

# Hack needed to ensure there's only one
_SLURP_SPEC_GENERATOR = None

class SpecCompiler(object):
    def __init__(self, spec_filename=None):
        self.proj = project.Project()

        if spec_filename is not None:
            self.loadSpec(spec_filename)

    def loadSpec(self,spec_filename):
        """
        Load the project object
        """
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

    def loadSimpleSpec(self,text="", regionList=[], sensors=[], actuators=[], customs=[], adj=[], outputfile=""):
        """
        Load a simple spec given by the arguments without reading from a spec file
        
        For Slurp

        region, sensors, actuators, customs are lists of strings representing props
        adj is a list of tuples [(region1,region2),...]
        """

        if outputfile is "":
            print "need to specify output filename"
            return

        self.proj.compile_options['decompose'] = False
        self.proj.project_root = os.path.abspath(os.path.dirname(os.path.expanduser(outputfile)))
        self.proj.project_basename, ext = os.path.splitext(os.path.basename(outputfile))
        self.proj.specText=text
        # construct a list of region objects with given names
        self.proj.rfi = regions.RegionFileInterface()
        for rname in regionList:
            self.proj.rfi.regions.append(regions.Region(name=rname))

        self.proj.enabled_sensors = sensors
        self.proj.enabled_actuators = actuators
        self.proj.all_customs = customs

        # construct adjacency matrix
        self.proj.rfi.transitions= [[[] for j in range(len(self.proj.rfi.regions))] for i in range(len(self.proj.rfi.regions))]
        for tran in adj:
            idx0 = self.proj.rfi.indexOfRegionWithName(tran[0])
            idx1 = self.proj.rfi.indexOfRegionWithName(tran[1])
            self.proj.rfi.transitions[idx0][idx1] = [(0,0)] # fake trans face
            self.proj.rfi.transitions[idx1][idx0] = [(0,0)]

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
        if self.proj.compile_options["decompose"]:
            numRegions = len(self.parser.proj.rfi.regions)
        else:
            numRegions = len(self.proj.rfi.regions)
        sensorList = self.proj.enabled_sensors
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs + self.proj.internal_props

        createSMVfile(self.proj.getFilenamePrefix(), numRegions, sensorList, robotPropList)

    def _writeLTLFile(self):
        #regionList = [r.name for r in self.parser.proj.rfi.regions]
        regionList = [r.name for r in self.proj.rfi.regions]
        sensorList = copy.deepcopy(self.proj.enabled_sensors)
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs
        
        text = self.proj.specText

        response = None

        # Create LTL using selected parser
        # TODO: rename decomposition object to something other than 'parser'
        if self.proj.compile_options["parser"] == "slurp":
            # Hack: We need to make sure there's only one of these
            global _SLURP_SPEC_GENERATOR
            
            # Make a new specgenerator and have it process the text
            if not _SLURP_SPEC_GENERATOR:
                # Add SLURP to path for import
                p = os.path.dirname(os.path.abspath(__file__))
                sys.path.append(os.path.join(p, "..", "etc", "SLURP"))
                from ltlbroom.specgeneration import SpecGenerator
                _SLURP_SPEC_GENERATOR = SpecGenerator()
            
            # Filter out regions it shouldn't know about
            filtered_regions = [region.name for region in self.proj.rfi.regions 
                                if not (region.isObstacle or region.name.lower() == "boundary")]
            LTLspec_env, LTLspec_sys, self.proj.internal_props, internal_sensors, responses, traceback = \
                _SLURP_SPEC_GENERATOR.generate(text, sensorList, filtered_regions, robotPropList,
                                               self.proj.currentConfig.region_tags)

            for ln, response in enumerate(responses):
                if not response:
                    print "WARNING: Could not parse the sentence in line {0}".format(ln)

            # Abort compilation if there were any errors
            if not all(responses):
                return None, responses
        
            # Add in the sensors so they go into the SMV and spec files
            for s in internal_sensors:
                if s not in sensorList:
                    sensorList.append(s)
                    self.proj.all_sensors.append(s)
                    self.proj.enabled_sensors.append(s)                    

            # Conjoin all the spec chunks
            LTLspec_env = '\t\t' + ' & \n\t\t'.join(LTLspec_env)
            LTLspec_sys = '\t\t' + ' & \n\t\t'.join(LTLspec_sys)
            
            if self.proj.compile_options["decompose"]:
                # substitute decomposed region names
                for r in self.proj.rfi.regions:
                    if not (r.isObstacle or r.name.lower() == "boundary"):
                        LTLspec_env = re.sub('\\bs\.' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_env)
                        LTLspec_env = re.sub('\\be\.' + r.name + '\\b', "("+' | '.join(["e."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_env)
                        LTLspec_sys = re.sub('\\bs\.' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_sys)
                        LTLspec_sys = re.sub('\\be\.' + r.name + '\\b', "("+' | '.join(["e."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_sys)

        elif self.proj.compile_options["parser"] == "ltl":
            # delete comments
            text = re.sub(r"#.*$", "", text, flags=re.MULTILINE)

            # split into env and sys parts (by looking for a line of just dashes in between)
            LTLspec_env, LTLspec_sys = re.split(r"^\s*-+\s*$", text, maxsplit=1, flags=re.MULTILINE)

            # split into subformulas
            LTLspec_env = re.split(r"(?:[ \t]*[\n\r][ \t]*)+", LTLspec_env)
            LTLspec_sys = re.split(r"(?:[ \t]*[\n\r][ \t]*)+", LTLspec_sys)

            # remove any empty initial entries (HACK?)
            while '' in LTLspec_env:
                LTLspec_env.remove('')
            while '' in LTLspec_sys:
                LTLspec_sys.remove('')

            print LTLspec_env
            print LTLspec_sys

            # automatically conjoin all the subformulas
            LTLspec_env = '\t\t' + ' & \n\t\t'.join(LTLspec_env)
            LTLspec_sys = '\t\t' + ' & \n\t\t'.join(LTLspec_sys)

            # substitute decomposed region 
            for r in self.proj.rfi.regions:
                if not (r.isObstacle or r.name.lower() == "boundary"):
                    LTLspec_env = re.sub('\\b' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_env)
                    LTLspec_sys = re.sub('\\b' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_sys)

            traceback = [] # HACK: needs to be something other than None
        elif self.proj.compile_options["parser"] == "structured":
            import parseEnglishToLTL

            # substitute decomposed region 
            for r in self.proj.rfi.regions:
                if not (r.isObstacle or r.name.lower() == "boundary"):
                    text = re.sub('\\b' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", text)

            regionList = ["s."+x.name for x in self.parser.proj.rfi.regions]

            spec, traceback, failed = parseEnglishToLTL.writeSpec(text, sensorList, regionList, robotPropList)

            # Abort compilation if there were any errors
            if failed:
                return None

            LTLspec_env = spec["EnvInit"] + spec["EnvTrans"] + spec["EnvGoals"]
            LTLspec_sys = spec["SysInit"] + spec["SysTrans"] + spec["SysGoals"]

        else:
            print "Parser type '{0}' not currently supported".format(self.proj.compile_options["parser"])
            return None


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

        if self.proj.compile_options["decompose"]:
            adjData = self.parser.proj.rfi.transitions
        else:
            adjData = self.proj.rfi.transitions
        ##############################################################################
        ######### BEGIN HACK: generate env topology info for follow scenario #########
        ##############################################################################

        # taken from createJTLVInput

        env_topology = []
        # The topological relation (adjacency)
        for Origin in range(len(adjData)):
            # from region i we can stay in region i
            env_topology.append('\t\t\t []( (')
            env_topology.append(currBitEnc[Origin])
            env_topology.append(') -> ( (')
            env_topology.append(nextBitEnc[Origin])
            env_topology.append(')')
            
            for dest in range(len(adjData)):
                if adjData[Origin][dest]:
                    # not empty, hence there is a transition
                    env_topology.append('\n\t\t\t\t\t\t\t\t\t| (')
                    env_topology.append(nextBitEnc[dest])
                    env_topology.append(') ')

            # closing this region
            env_topology.append(' ) ) & \n ')

        env_topology = ''.join(env_topology).replace("s.bit", "e.sbit")
        
        # Setting the system initial formula to allow only valid
        #  region encoding. This may be redundent if an initial region is
        #  specified, but it is here to ensure the system cannot start from
        #  an invalid encoding
        initreg_formula = '\t\t\t( ' + currBitEnc[0] + ' \n'
        for regionInd in range(1,len(currBitEnc)):
            initreg_formula = initreg_formula + '\t\t\t\t | ' + currBitEnc[regionInd] + '\n'
        initreg_formula = initreg_formula + '\t\t\t) \n'
        initreg_formula = initreg_formula.replace("s.bit", "e.sbit")

        if "FOLLOW_SENSOR_CONSTRAINTS" in LTLspec_env:
            sensorBits = ["sbit{0}".format(n) for n in range(0,numBits)]
            for p in sensorBits:
                if p not in self.proj.enabled_sensors:
                    self.proj.enabled_sensors.append(p)
                if p not in self.proj.all_sensors:   
                    self.proj.all_sensors.append(p)

        LTLspec_env = LTLspec_env.replace("FOLLOW_SENSOR_CONSTRAINTS", env_topology + initreg_formula)

        ##############################################################################
        #################################### END HACK ################################
        ##############################################################################

        createLTLfile(self.proj.getFilenamePrefix(), sensorList, robotPropList, adjData, LTLspec_env, LTLspec_sys)

        response = responses

        return traceback, response
        
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
            proj_copy = copy.deepcopy(self.proj)
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
        if self.proj.compile_options["decompose"]:
            print "--> Decomposing..."
            self._decompose()
        print "--> Writing LTL file..."
        tb, resp = self._writeLTLFile()
        print "--> Writing SMV file..."
        self._writeSMVFile()

        if tb is None:
            print "ERROR: Compilation aborted"
            return 

        #self._checkForEmptyGaits()
        print "--> Synthesizing..."
        return self._synthesize(with_safety_aut)

