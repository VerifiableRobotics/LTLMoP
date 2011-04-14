#!/usr/bin/env python

""" ================================================
    project.py - Abstraction layer for project files
    ================================================
    
    This module exposes an object that allows for simplified loading of the
    various files included in a single project.
"""

# TODO: Document better
# TODO: Add in output (write-to-file) functions?

import os, sys
import fileMethods, regions
from numpy import *

class Project:
    """
    A project object.
    """

    def __init__(self):
        self.project_basename = None
        self.silent = False
        pass

    def setSilent(self, silent):
        self.silent = silent

    def getExperimentConfig(self, exp_cfg_name):
        """
        Returns a dictionary corresponding to the specified experiment config
        """

        # Find the section that corresponds to this configuration
        for key, val in self.spec_data.iteritems():
            if key.startswith("EXPERIMENT CONFIG") and val['Name'][0] == exp_cfg_name :
                if not self.silent: print "  -> Using experiment configuration \"%s\"" % exp_cfg_name
                return val


        if not self.silent: print "ERROR: Could not find experiment config with name \"%s\" in spec file!" % exp_cfg_name
        return

    def loadLabData(self, exp_cfg_data):
        """
        Takes an experiment config dictionary and returns a lab config dictionary.
        """

        #### Load in the lab setup file
        
        try:
            lab_name = exp_cfg_data['Lab'][0]
        except IndexError, KeyError:
            if not self.silent: print "WARNING: Lab configuration file undefined"        
            return

        # Add extension to the name if there isn't one. 
        if not lab_name.endswith('.lab'):
            lab_name = lab_name+'.lab'     
        if not self.silent: print "Loading lab setup file %s..." % lab_name
        try:
            # First try path relative to project path
            lab_data = fileMethods.readFromFile(os.path.join(self.project_root, lab_name))   
        except IOError: 
            try:
                # If that doesn't work, try looking in $self.ltlmop_root/labs/ directory
                lab_data = fileMethods.readFromFile(os.path.join(self.ltlmop_root, "labs", lab_name))   
            except IOError:
                if not self.silent: print "ERROR: Couldn't find lab setup file in project directory or labs folder."
                return
        if not self.silent: print "  -> Looks like you want to run your experiment with %s. Good choice." % lab_data["Name"][0]
        
        return lab_data
        
    def loadRegionMapping(self, spec_data):
        """
        Takes the region mapping data and returns region mapping dictionary.
        """
        try:
            mapping_data = spec_data['SPECIFICATION']['RegionMapping']
        except IndexError, KeyError:
            if not self.silent: print "WARNING: Region mapping data undefined"        
            return {'Null':['Null']}
        if len(mapping_data) == 0:
            if not self.silent: print "WARNING: Region mapping data is empty"        
            return {'Null':['Null']}
        
        regionMapping = {}
        for line in mapping_data:
            oldRegionName,newRegionList = line.split('=')
            regionMapping[oldRegionName] = newRegionList.split(',')
        return regionMapping
        
        
    def loadRobotFile(self, exp_cfg_data):
        """
        Takes an experiment config dictionary and returns a robot description dictionary.
        """

        #### Load in the robot file
        
        try:
            rdf_name = exp_cfg_data['RobotFile'][0]
        except IndexError, KeyError:
            if not self.silent: print "WARNING: Robot description file undefined"        
            return

        # Add extension to the name if there isn't one. 
        if not rdf_name.endswith('.robot'):
            rdf_name = rdf_name+'.robot'  
     
        if not self.silent: print "Loading robot description file %s..." % rdf_name
        try:
            # First try path relative to project path
            rdf_data = fileMethods.readFromFile(os.path.join(self.project_root, rdf_name))   
        except IOError: 
            try:
                # If that doesn't work, try looking in $self.ltlmop_root/robots/ directory
                rdf_data = fileMethods.readFromFile(os.path.join(self.ltlmop_root, "robots", rdf_name))   
            except IOError:
                if not self.silent: print "ERROR: Couldn't find robot description file in project directory or robots folder."
                return
        if not self.silent: print "  -> %s looks excited for this run." % rdf_data["Name"][0]
        
        return rdf_data

    def loadRegionFile(self, decomposed=False):
        """
        Returns a Region File Interface object corresponding to the regions file referenced in the spec file
        """

        #### Load in the region file

        try:
            regf_name = self.spec_data['SETTINGS']['RegionFile'][0]
            if decomposed:
                #regf_name = regf_name.split(".")[0] + "_decomposed.regions"
                regf_name = self.getFilenamePrefix() + "_decomposed.regions"
        except IndexError, KeyError:
            if not self.silent: print "WARNING: Region file undefined"        
            return

        if not self.silent: print "Loading region file %s..." % regf_name
        rfi = regions.RegionFileInterface() 

        if rfi.readFile(os.path.join(self.project_root, regf_name)) == False:
            if not self.silent:
                print "ERROR: Could not load region file %s!"  % regf_name
                if decomposed:
                    print "Are you sure you compiled your specification?"
            return
     
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
        coordmap_map2lab = lambda pt: (dot(scale, pt) + offset)
        coordmap_lab2map = lambda pt: (dot(inv_scale, pt - offset))

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
                sys.exit(1)

        self.ltlmop_root = os.path.join(p,"src")

        ### Load in the specification file
        if not self.silent: print "Loading specification file %s..." % spec_file
        spec_data = fileMethods.readFromFile(spec_file)   
        
        return spec_data
    
    def loadProject(self, spec_file, exp_cfg_name=None):
        """
        Because the spec_file contains references to all other project files, this is all we
        need to know in order to load everything in.
        """

        self.spec_data = self.loadSpecFile(spec_file)

        # Figure out the name of the current experiment config if not specified
        if exp_cfg_name is None:
            exp_cfg_name = self.spec_data['SETTINGS']['currentExperimentName'][0]

        self.regionMapping = self.loadRegionMapping(self.spec_data)
        self.exp_cfg_data = self.getExperimentConfig(exp_cfg_name)
        self.lab_data = self.loadLabData(self.exp_cfg_data)
        self.robot_data = self.loadRobotFile(self.exp_cfg_data)
        self.rfi = self.loadRegionFile()
        self.coordmap_map2lab, self.coordmap_lab2map = self.getCoordMaps(self.exp_cfg_data)
        self.determineEnabledPropositions()

    def determineEnabledPropositions(self):
        """
        Populate ``all_sensors``, ``initial_sensors``, and ``all_actuators`` lists based on
        configuration information.
        """
    
        # Figure out what sensors are enabled, and which are initially true
        self.all_sensors = []
        self.initial_sensors = []
        for line in self.spec_data['SETTINGS']['Sensors']:
            sensor, val = line.split(',')
            if int(val) == 1: 
                self.all_sensors.append(sensor)
                if sensor in self.exp_cfg_data['InitialTruths']:
                    self.initial_sensors.append(sensor)

        # Figure out what actuators are enabled
        self.all_actuators = []
        for line in self.spec_data['SETTINGS']['Actions']:
            act, val = line.split(',')
            if int(val) == 1: 
                self.all_actuators.append(act)

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
        return self.rfi.thumb
        #return self.getFilenamePrefix() + "_simbg.png"
    
    def lookupHandlers(self):
        """
        Figure out which handlers we are going to use, based on the different configurations file settings
        """

        # TODO: Complain nicely instead of just dying when this breaks?
        self.h_name = {}
        self.h_name['init'] = self.lab_data["InitializationHandler"]
        self.h_name['pose'] = self.lab_data["PoseHandler"][0]
        self.h_name['sensor'] = self.lab_data["SensorHandler"][0]
        self.h_name['actuator'] = self.lab_data["ActuatorHandler"][0]
        self.h_name['locomotionCommand'] = self.lab_data["LocomotionCommandHandler"][0]
        self.h_name['motionControl'] = self.robot_data["MotionControlHandler"][0]
        self.h_name['drive'] = self.robot_data["DriveHandler"][0]
    
    def runInitialization(self, calib=False):
        """
        Run the necessary initialization handlers.

        We treat initialization handlers separately, because there may be more than one.
        NOTE: These will be loaded in the same order as they are listed in the lab config file.
        """

        self.shared_data = {}  # This is for storing things like server connection objects, etc.
        init_num = 1
        self.init_handlers = []
        sys.path.append(self.ltlmop_root)  # Temporary fix until paths get straightened out
        for handler in self.h_name['init']:
            if not self.silent: print "  -> %s" % handler
            # TODO: Is there a more elegant way to do this? This is pretty ugly...
            exec("from %s import initHandler as initHandler%d" % (handler, init_num)) in locals() # WARNING: This assumes our input data is not malicious...
            exec("self.init_handlers.append(initHandler%d(self, calib=calib))" % (init_num)) in locals()
            self.shared_data.update(self.init_handlers[-1].getSharedData())
            init_num += 1  # So they don't clobber each other

        return self.shared_data

    def importHandlers(self, list=None):
        """
        Load in specified handlers.  If no list is given, *all* handlers will be loaded.

        Note that the order of loading is important, due to inter-handler dependencies.
        """

        if list is None:
            list = ['pose','sensor','actuator','locomotionCommand','drive','motionControl']

        sys.path.append(self.ltlmop_root)  # Temporary fix until paths get straightened out
        # Now do the rest of them
        for handler in list:
            if not self.silent: print "  -> %s" % self.h_name[handler]
            exec("from %s import %sHandler" % (self.h_name[handler], handler)) in locals() # WARNING: This assumes our input data is not malicious...
            if handler == 'pose':
                self.pose_handler = poseHandler(self, self.shared_data)
                if not self.silent: print "(POSE) Initial pose: " + str(self.pose_handler.getPose())
            elif handler == 'sensor':
                self.sensor_handler = sensorHandler(self, self.shared_data)
            elif handler == 'actuator':
                self.actuator_handler = actuatorHandler(self, self.shared_data)
            elif handler == 'locomotionCommand':
                self.loco_handler = locomotionCommandHandler(self, self.shared_data)
            elif handler == 'drive':
                self.drive_handler = driveHandler(self, self.shared_data)
            elif handler == 'motionControl':
                self.motion_handler = motionControlHandler(self, self.shared_data)
