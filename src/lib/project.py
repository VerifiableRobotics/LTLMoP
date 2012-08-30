#!/usr/bin/env python

""" ================================================
    project.py - Abstraction layer for project files
    ================================================

    This module exposes an object that allows for simplified loading of the
    various files included in a single project.
"""

# TODO: Document better

import os, sys
import fileMethods, regions
from numpy import *
import handlerSubsystem
import inspect

class Project:
    """
    A project object.
    """

    def __init__(self):
        self.project_basename = None
        self.project_root = None
        self.spec_data = None
        self.silent = False
        self.regionMapping = None
        self.rfi = None
        self.specText = ""
        self.all_sensors = []
        self.enabled_sensors = []
        self.all_actuators = []
        self.enabled_actuators = []
        self.all_customs = []
        self.internal_props = []
        self.currentConfig = None
        self.shared_data = {}  # This is for storing things like server connection objects, etc.

        self.h_instance = {'init':{},'pose':None,'locomotionCommand':None,'motionControl':None,'drive':None,'sensor':{},'actuator':{}}

        # Compilation options (with defaults)
        self.compile_options = {"convexify": True,  # Decompose workspace into convex regions
                                "fastslow": False}  # Enable "fast-slow" synthesis algorithm

        # Climb the tree to find out where we are
        p = os.path.abspath(sys.argv[0])
        t = ""
        while t != "src":
            (p, t) = os.path.split(p)
            if p == "":
                print "I have no idea where I am; this is ridiculous"
                return None

        self.ltlmop_root = os.path.join(p, "src")

    def setSilent(self, silent):
        self.silent = silent

    def loadRegionMapping(self):
        """
        Takes the region mapping data and returns region mapping dictionary.
        """

        if self.spec_data is None:
            if not self.silent: print "ERROR: Cannot load region mapping data before loading a spec file"
            return None

        try:
            mapping_data = self.spec_data['SPECIFICATION']['RegionMapping']
        except KeyError:
            if not self.silent: print "WARNING: Region mapping data undefined"
            return None

        if len(mapping_data) == 0:
            if not self.silent: print "WARNING: Region mapping data is empty"
            return None

        regionMapping = {}
        for line in mapping_data:
            oldRegionName, newRegionList = line.split('=')
            regionMapping[oldRegionName.strip()] = [n.strip() for n in newRegionList.split(',')]

        return regionMapping

    def loadRegionFile(self, decomposed=False):
        """
        Returns a Region File Interface object corresponding to the regions file referenced in the spec file
        """

        #### Load in the region file

        if decomposed:
            regf_name = self.getFilenamePrefix() + "_decomposed.regions"
        else:
            try:
                regf_name = os.path.join(self.project_root, self.spec_data['SETTINGS']['RegionFile'][0])
            except (IndexError, KeyError):
                if not self.silent: print "WARNING: Region file undefined"
                return None

        if not self.silent: print "Loading region file %s..." % regf_name
        rfi = regions.RegionFileInterface()

        if not rfi.readFile(regf_name):
            if not self.silent:
                print "ERROR: Could not load region file %s!"  % regf_name
                if decomposed:
                    print "Are you sure you compiled your specification?"
            return None

        if not self.silent: print "  -> Found definitions for %d regions." % len(rfi.regions)

        return rfi

    def getCoordMaps(self):
        """
        Returns forward (map->lab) and reverse (lab->map) coordinate mapping functions, in that order
        """

        if self.currentConfig is None:
            return (None, None)

        r = self.currentConfig.getRobotByName(self.currentConfig.main_robot)
        if r.calibrationMatrix is None:
            if not self.silent: print "WARNING: Main robot has no calibration data.  Using identity matrix."
            T = eye(3)
        else:
            T = r.calibrationMatrix

        # Check for singular matrix
        if abs(linalg.det(T)) < finfo(float).eps:
            if not self.silent: print "WARNING: Singular calibration matrix.  Ignoring, and using identity matrix."
            T = eye(3)

        #### Create the coordmap functions
        coordmap_map2lab = lambda pt: (linalg.inv(T) * mat([pt[0], pt[1], 1]).T).T.tolist()[0][0:2]
        coordmap_lab2map = lambda pt: (T * mat([pt[0], pt[1], 1]).T).T.tolist()[0][0:2]

        return coordmap_map2lab, coordmap_lab2map

    def loadSpecFile(self, spec_file):
        # Figure out where we should be looking for files, based on the spec file name & location
        self.project_root = os.path.abspath(os.path.dirname(spec_file))
        self.project_basename, ext = os.path.splitext(os.path.basename(spec_file))


        ### Load in the specification file
        if not self.silent: print "Loading specification file %s..." % spec_file
        spec_data = fileMethods.readFromFile(spec_file)

        if spec_data is None:
            if not self.silent: print "WARNING: Failed to load specification file"
            return None

        try:
            self.specText = '\n'.join(spec_data['SPECIFICATION']['Spec'])
        except KeyError:
            if not self.silent: print "WARNING: Specification text undefined"

        if 'CompileOptions' in spec_data['SETTINGS']:
            for l in spec_data['SETTINGS']['CompileOptions']:
                if ":" not in l:
                    continue

                k,v = l.split(":", 1)
                self.compile_options[k.strip().lower()] = (v.strip().lower() in ['true', 't', '1'])

        return spec_data

    def writeSpecFile(self, filename=None):
        if filename is None:
            # Default to same filename as we loaded from
            filename = os.path.join(self.project_root, self.project_basename + ".spec")
        else:
            # Update our project paths based on the new filename
            self.project_root = os.path.dirname(os.path.abspath(filename))
            self.project_basename, ext = os.path.splitext(os.path.basename(filename))

        data = {}

        data['SPECIFICATION'] = {"Spec": self.specText}

        if self.regionMapping is not None:
            data['SPECIFICATION']['RegionMapping'] = [rname + " = " + ', '.join(rlist) for
                                                      rname, rlist in self.regionMapping.iteritems()]

        data['SETTINGS'] = {"Sensors": [p + ", " + str(int(p in self.enabled_sensors)) for p in self.all_sensors],
                            "Actions": [p + ", " + str(int(p in self.enabled_actuators)) for p in self.all_actuators],
                            "Customs": self.all_customs}

        if self.currentConfig is not None:
            data['SETTINGS']['CurrentConfigName'] = self.currentConfig.name

        data['SETTINGS']['CompileOptions'] = "\n".join(["%s: %s" % (k, str(v)) for k,v in self.compile_options.iteritems()])
    
        if self.rfi is not None:
            # Save the path to the region file as relative to the spec file
            # FIXME: relpath has case sensitivity problems on OS X
            data['SETTINGS']['RegionFile'] = os.path.normpath(os.path.relpath(self.rfi.filename, self.project_root))

        comments = {"FILE_HEADER": "This is a specification definition file for the LTLMoP toolkit.\n" +
                                   "Format details are described at the beginning of each section below.",
                    "RegionFile": "Relative path of region description file",
                    "Sensors": "List of sensor propositions and their state (enabled = 1, disabled = 0)",
                    "Actions": "List of action propositions and their state (enabled = 1, disabled = 0)",
                    "Customs": "List of custom propositions",
                    "Spec": "Specification in structured English",
                    "RegionMapping": "Mapping between region names and their decomposed counterparts"}

        fileMethods.writeToFile(filename, data, comments)

    def loadConfig(self, name=None):
        """
        Load the config object with name ``name`` (case-insensitive).  If no name is specified, load the one defined as currently selected.
        """

        self.hsub = handlerSubsystem.HandlerSubsystem(self)
        self.hsub.setSilent(self.silent)
        self.hsub.loadAllConfigFiles()

        if name is None:
            try:
                name = self.spec_data['SETTINGS']['CurrentConfigName'][0]
            except (KeyError, IndexError):
                if not self.silent: print "WARNING: No experiment configuration defined"
                return None

        for c in self.hsub.configs:
            if c.name.lower() == name.lower():
                return c

        if not self.silent: print "WARNING: Default experiment configuration of name '%s' could not be found in configs/ directory." % name

        return None

    def loadProject(self, spec_file):
        """
        Because the spec_file contains references to all other project files, this is all we
        need to know in order to load everything in.
        """

        self.spec_data = self.loadSpecFile(spec_file)

        if self.spec_data is None:
            return False

        self.currentConfig = self.loadConfig()
        self.regionMapping = self.loadRegionMapping()
        self.rfi = self.loadRegionFile()
        self.coordmap_map2lab, self.coordmap_lab2map = self.getCoordMaps()
        self.determineEnabledPropositions()

        return True

    def determineEnabledPropositions(self):
        """
        Populate lists ``all_sensors``, ``enabled_sensors``, etc.
        """

        # Figure out what sensors are enabled
        self.all_sensors = []
        self.enabled_sensors = []
        for line in self.spec_data['SETTINGS']['Sensors']:
            sensor, val = line.split(',')
            self.all_sensors.append(sensor.strip())
            if int(val) == 1:
                self.enabled_sensors.append(sensor.strip())

        # Figure out what actuators are enabled
        self.all_actuators = []
        self.enabled_actuators = []
        for line in self.spec_data['SETTINGS']['Actions']:
            act, val = line.split(',')
            self.all_actuators.append(act.strip())
            if int(val) == 1:
                self.enabled_actuators.append(act.strip())

        # Figure out what the custom propositions are
        self.all_customs = self.spec_data['SETTINGS']['Customs']

    def getFilenamePrefix(self):
        """ Returns the full path of most project files, minus the extension.

            For example, if the spec file of this project is ``/home/ltlmop/examples/test/test.spec``
            then this function will return ``/home/ltlmop/examples/test/test``
        """
        return os.path.join(self.project_root, self.project_basename)

    def importHandlers(self, all_handler_types=None):
        """
        Figure out which handlers we are going to use, based on the different configurations file settings
        Only one motion/pose/drive/locomotion handler per experiment
        Multiple init/sensor/actuator handlers per experiment, one for each robot (if any)
        Load in specified handlers.  If no list is given, *all* handlers will be loaded.
        Note that the order of loading is important, due to inter-handler dependencies.
        """

        if all_handler_types is None:
            all_handler_types = ['init','pose','locomotionCommand','drive','motionControl','sensor','actuator']

        self.hsub.importHandlers(self.currentConfig,all_handler_types)
        if not self.silent: print "(POSE) Initial pose: " + str(self.h_instance['pose'].getPose())
