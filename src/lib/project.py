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
        self.currentConfig = None

    def setSilent(self, silent):
        self.silent = silent

    def loadRegionMapping(self):
        """
        Takes the region mapping data and returns region mapping dictionary.
        """

        if self.spec_data is None:
            print "ERROR: Cannot load region mapping data before loading a spec file"
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

    def getCoordMaps(self, exp_cfg_data):
        """
        Returns forward (map->lab) and reverse (lab->map) coordinate mapping functions, in that order

        We are currently assuming the transformation is only linear, not affine (TODO).
        """

        #### Create the coordmap functions

        # Look for transformation values in spec file
        try:
            transformValues = exp_cfg_data['Calibration'][0].split(",")
            [xscale, xoffset, yscale, yoffset] = map(float, transformValues)
        except KeyError, ValueError:
            if not self.silent: print "ERROR: Please calibrate and update values before running simulation."
            return

        # Create functions for coordinate transformation
        # (numpy may seem like overkill for this, but we already have it as a dependency anyways...)
        scale = diag([xscale, yscale])
        inv_scale = linalg.inv(scale)
        offset = array([xoffset, yoffset])

        #coordmap_map2lab = lambda pt: (array((a*vstack([mat(pt).T,1]))[0:2]).flatten()) 
        #coordmap_lab2map = lambda pt: (array((linalg.inv(a)*vstack([mat(pt).T,1]))[0:2]).flatten()) 

        coordmap_map2lab = lambda pt: (dot(scale, array([pt[0], pt[1]])) + offset)
        coordmap_lab2map = lambda pt: (dot(inv_scale, array([pt[0], pt[1]]) - offset))

        return coordmap_map2lab, coordmap_lab2map

    def loadSpecFile(self, spec_file):
        # Figure out where we should be looking for files, based on the spec file name & location
        self.project_root = os.path.abspath(os.path.dirname(spec_file))
        self.project_basename, ext = os.path.splitext(os.path.basename(spec_file)) 
        
        # Climb the tree to find out where we are
        p = os.path.abspath(sys.argv[0])
        t = ""
        while t != "src":
            (p, t) = os.path.split(p)
            if p == "":
                print "I have no idea where I am; this is ridiculous"
                return None

        self.ltlmop_root = os.path.join(p, "src")

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

    def loadProject(self, spec_file, current_config_name=None):
        """
        Because the spec_file contains references to all other project files, this is all we
        need to know in order to load everything in.
        """

        self.spec_data = self.loadSpecFile(spec_file)

        # Figure out the name of the current experiment config if not specified
        if current_config_name is None:
            try:
                current_config_name = self.spec_data['SETTINGS']['CurrentConfigName'][0]
            except (KeyError, IndexError):
                if not self.silent: print "WARNING: No experiment configuration defined"        
                self.currentConfig = None
            else:
                self.currentConfig = self.loadExperimentConfig(current_config_name.replace(" ", "_"))

        self.regionMapping = self.loadRegionMapping()
        self.rfi = self.loadRegionFile()
        #self.coordmap_map2lab, self.coordmap_lab2map = self.getCoordMaps(self.exp_cfg_data)
        self.determineEnabledPropositions()
        
        
    def loadExperimentConfig(self, configFileName):
        self.hSub = handlerSubsystem.HandlerSubsystem(self)
        configObj = self.hSub.config_parser.loadConfigFile(configFileName)
        return configObj

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

    def getBackgroundImagePath(self):
        """ Returns the path of the background image with regions drawn on top, created by RegionEditor """
        
        # TODO: remove this and all use of bg image png
        return self.rfi.thumb


    def importHandlers(self, all_handler_types=None):
        """
        Figure out which handlers we are going to use, based on the different configurations file settings
        Only one motion/pose/drive/locomotion handler per experiment
        Multiple init/sensor/actuator handlers per experiment, one for each robot (if any)
        Load in specified handlers.  If no list is given, *all* handlers will be loaded.
        Note that the order of loading is important, due to inter-handler dependencies.
        """

        if all_handler_types is None:
            all_handler_types = ['init','pose','sensor','actuator','locomotionCommand','drive','motionControl']

        self.shared_data = {}  # This is for storing things like server connection objects, etc.
        self.h_instance = {'init':{},'pose':None,'locomotionCommand':None,'motionControl':None,'drive':None,'sensor':{},'actuator':{}}

        self.hSub.importHandlers(self.current_configObj,all_handler_types)
        self.pose_handler = self.h_instance['pose']
        if not self.silent: print "(POSE) Initial pose: " + str(self.pose_handler.getPose())
        self.loco_handler = self.h_instance['locomotionCommand']
        self.drive_handler = self.h_instance['drive']
        self.motion_handler = self.h_instance['motionControl']
            
            
#            exec("from %s import %sHandler" % (self.h_name[handler], handler)) in locals() # WARNING: This assumes our input data is not malicious...
#            if handler == 'pose':
#                self.pose_handler = poseHandler(self, self.shared_data)
#                if not self.silent: print "(POSE) Initial pose: " + str(self.pose_handler.getPose())
#            elif handler == 'sensor':
#                self.sensor_handler = sensorHandler(self, self.shared_data)
#            elif handler == 'actuator':
#                self.actuator_handler = actuatorHandler(self, self.shared_data)
#            elif handler == 'locomotionCommand':
#                self.loco_handler = locomotionCommandHandler(self, self.shared_data)
#            elif handler == 'drive':
#                self.drive_handler = driveHandler(self, self.shared_data)
#            elif handler == 'motionControl':
#                self.motion_handler = motionControlHandler(self, self.shared_data)
#                
#                
                
                
   

#    def lookupHandlers(self):
#        """
#        Figure out which handlers we are going to use, based on the different configurations file settings
#        """

#        # TODO: Complain nicely instead of just dying when this breaks?
#        self.h_name = {}
#        self.h_name['init'] = self.lab_data["InitializationHandler"]
#        self.h_name['pose'] = self.lab_data["PoseHandler"][0]
#        self.h_name['sensor'] = self.lab_data["SensorHandler"][0]
#        self.h_name['actuator'] = self.lab_data["ActuatorHandler"][0]
#        self.h_name['locomotionCommand'] = self.lab_data["LocomotionCommandHandler"][0]
#        self.h_name['motionControl'] = self.robot_data["MotionControlHandler"][0]
#        self.h_name['drive'] = self.robot_data["DriveHandler"][0]   
    
#    def runInitialization(self, calib=False):
#        """
#        Run the necessary initialization handlers.

#        We treat initialization handlers separately, because there may be more than one.
#        NOTE: These will be loaded in the same order as they are listed in the lab config file.
#        """

#        self.shared_data = {}  # This is for storing things like server connection objects, etc.
#        init_num = 1
#        self.init_handlers = []
#        sys.path.append(self.ltlmop_root)  # Temporary fix until paths get straightened out
#        for handler in self.h_name['init']:
#            if not self.silent: print "  -> %s" % handler
#            # TODO: Is there a more elegant way to do this? This is pretty ugly...
#            exec("from %s import initHandler as initHandler%d" % (handler, init_num)) in locals() # WARNING: This assumes our input data is not malicious...
#            exec("self.init_handlers.append(initHandler%d(self, calib=calib))" % (init_num)) in locals()
#            self.shared_data.update(self.init_handlers[-1].getSharedData())
#            init_num += 1  # So they don't clobber each other

#        return self.shared_data



#    def importHandlers(self, list=None):
#        """
#        Load in specified handlers.  If no list is given, *all* handlers will be loaded.

#        Note that the order of loading is important, due to inter-handler dependencies.
#        """

#        if list is None:
#            list = ['pose','sensor','actuator','locomotionCommand','drive','motionControl']

#        sys.path.append(self.ltlmop_root)  # Temporary fix until paths get straightened out
#        # Now do the rest of them
#        for handler in list:
#            if not self.silent: print "  -> %s" % self.h_name[handler]
#            exec("from %s import %sHandler" % (self.h_name[handler], handler)) in locals() # WARNING: This assumes our input data is not malicious...
#            if handler == 'pose':
#                self.pose_handler = poseHandler(self, self.shared_data)
#                if not self.silent: print "(POSE) Initial pose: " + str(self.pose_handler.getPose())
#            elif handler == 'sensor':
#                self.sensor_handler = sensorHandler(self, self.shared_data)
#            elif handler == 'actuator':
#                self.actuator_handler = actuatorHandler(self, self.shared_data)
#            elif handler == 'locomotionCommand':
#                self.loco_handler = locomotionCommandHandler(self, self.shared_data)
#            elif handler == 'drive':
#                self.drive_handler = driveHandler(self, self.shared_data)
#            elif handler == 'motionControl':
#                self.motion_handler = motionControlHandler(self, self.shared_data)
#                
