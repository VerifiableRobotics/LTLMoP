import os, sys
import re
import time
import subprocess

sys.path.append("lib")

import project
import parseLP
from createJTLVinput import createLTLfile, createSMVfile
from parseEnglishToLTL import writeSpec

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
        
        # substitute the regions name in specs
        text = self.proj.specText
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
            text = self.proj.specText

        spec, traceback = writeSpec(text, sensorList, regionList, robotPropList)

        # TODO: Catch errors here
        adjData = self.parser.proj.rfi.transitions

        createLTLfile(self.proj.getFilenamePrefix(), sensorList, robotPropList, adjData, spec)

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

    def _synthesize(self, with_safety_aut=False):
        # Windows uses a different delimiter for the java classpath
        if os.name == "nt":
            delim = ";"
        else:
            delim = ":"

        classpath = delim.join([os.path.join(self.proj.ltlmop_root, "etc", "jtlv", "jtlv-prompt1.4.0.jar"), os.path.join(self.proj.ltlmop_root, "etc", "jtlv", "GROne")])

        cmd = ["java", "-ea", "-Xmx512m", "-cp", classpath, "GROneMain", self.proj.getFilenamePrefix() + ".smv", self.proj.getFilenamePrefix() + ".ltl"]

        # Generally used for Mopsy
        if with_safety_aut:
            cmd.append("--safety")

        subp = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=False)
        
        # TODO: Make this output live
        while subp.poll():
            time.sleep(0.1)

        realizable = False

        output = ""
        for line in subp.stdout:
            output += line
            if "Specification is realizable" in line:
               realizable = True
               
        subp.stdout.close()

        return (realizable, output)

    def compile(self, with_safety_aut=False):
        self._decompose()
        self._writeSMVFile()
        self._writeLTLFile()
        #self._checkForEmptyGaits()
        self._synthesize(with_safety_aut)

