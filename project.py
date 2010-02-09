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
    def __init__(self):
        pass

    def getExperimentConfig(self, exp_cfg_name):
        """
        Returns a dictionary corresponding to the specified experiment config
        """

        # Find the section that corresponds to this configuration
        for key, val in self.spec_data.iteritems():
            if key.startswith("EXPERIMENT CONFIG") and val['Name'][0] == exp_cfg_name:
                print "  -> Using experiment configuration \"%s\"" % exp_cfg_name
                return val

        print "ERROR: Could not find experiment config with name \"%s\" in spec file!" % exp_cfg_name
        sys.exit(0)

    def loadLabData(self, exp_cfg_data):
        """
        Takes an experiment config dictionary and returns a lab config dictionary.
        """

        #### Load in the lab setup file

        
        lab_name = exp_cfg_data['Lab'][0]
        # Add extension to the name if there isn't one. 
        if not lab_name.endswith('.lab'):
            lab_name = lab_name+'.lab'     
        print "Loading lab setup file %s..." % lab_name
        try:
            # First try path relative to project path
            lab_data = fileMethods.readFromFile(os.path.join(self.project_root, lab_name))   
        except IOError: 
            try:
                # If that doesn't work, try looking in $self.ltlmop_root/labs/ directory
                lab_data = fileMethods.readFromFile(os.path.join(self.ltlmop_root, "labs", lab_name))   
            except IOError:
                print "ERROR: Couldn't find lab setup file in project directory or labs folder."
                sys.exit(1)
        print "  -> Looks like you want to run your experiment with %s. Good choice." % lab_data["Name"][0]
        
        return lab_data

    def loadRobotFile(self, exp_cfg_data):
        """
        Takes an experiment config dictionary and returns a robot description dictionary.
        """

        #### Load in the robot file
        
        rdf_name = exp_cfg_data['RobotFile'][0]
        # Add extension to the name if there isn't one. 
        if not rdf_name.endswith('.robot'):
            rdf_name = rdf_name+'.robot'  
     
        print "Loading robot description file %s..." % rdf_name
        try:
            # First try path relative to project path
            rdf_data = fileMethods.readFromFile(os.path.join(self.project_root, rdf_name))   
        except IOError: 
            try:
                # If that doesn't work, try looking in $self.ltlmop_root/robots/ directory
                rdf_data = fileMethods.readFromFile(os.path.join(self.ltlmop_root, "robots", rdf_name))   
            except IOError:
                print "ERROR: Couldn't find robot description file in project directory or robots folder."
                sys.exit(1)
        print "  -> %s looks excited for this run." % rdf_data["Name"][0]
        
        return rdf_data

    def loadRegionFile(self):
        """
        Returns a Region File Interface object corresponding to the regions file referenced in the spec file
        """

        #### Load in the region file

        regf_name = self.spec_data['SETTINGS']['RegionFile'][0]

        print "Loading region file %s..." % regf_name
        rfi = regions.RegionFileInterface() 
        rfi.readFile(os.path.join(self.project_root, regf_name))
     
        print "  -> Found definitions for %d regions." % len(rfi.regions)

        return rfi

    def getCoordMaps(self, exp_cfg_data):
        """
        Returns forward and reverse coordinate mapping functions
        """

        #### Create the coordmap functions

        # Look for transformation values in spec file
        try:
            transformValues = exp_cfg_data['Calibration'][0].split(",")
            [xscale, xoffset, yscale, yoffset] = map(float, transformValues)
        except KeyError, ValueError:
            print "Error: Please calibrate and update values before running simulation."
            sys.exit(0)

        # Create functions for coordinate transformation
        # (numpy may seem like overkill for this, but we already have it as a dependency anyways...)
        scale = diag([xscale, yscale])
        inv_scale = linalg.inv(scale)
        offset = array([xoffset, yoffset])

        #fwd_coordmap = lambda pt: (array((a*vstack([mat(pt).T,1]))[0:2]).flatten()) 
        #rev_coordmap = lambda pt: (array((linalg.inv(a)*vstack([mat(pt).T,1]))[0:2]).flatten()) 
        fwd_coordmap = lambda pt: (dot(scale, pt) + offset)
        rev_coordmap = lambda pt: (dot(inv_scale, pt - offset))

        return fwd_coordmap, rev_coordmap

    def loadSpecFile(self, spec_file):
        # Figure out where we should be looking for files, based on the spec file name & location
        self.project_root = os.path.dirname(spec_file)
        self.project_basename, ext = os.path.splitext(os.path.basename(spec_file)) 
        self.ltlmop_root = os.path.dirname(sys.argv[0])

        ### Load in the specification file
        print "Loading specification file %s..." % spec_file
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

        self.exp_cfg_data = self.getExperimentConfig(exp_cfg_name)
        self.lab_data = self.loadLabData(self.exp_cfg_data)
        self.robot_data = self.loadRobotFile(self.exp_cfg_data)
        self.rfi = self.loadRegionFile()
        self.fwd_coordmap, self.rev_coordmap = self.getCoordMaps(self.exp_cfg_data)
    
    def getBackgroundImagePath(self):
        img_file = os.path.join(self.project_root, self.project_basename) + "_simbg.png"
        return img_file
    
    def lookupHandlers(self):
        # Figure out which handlers we are going to use
        # TODO: Complain nicely instead of just dying when this breaks?
        self.h_name = {}
        self.h_name['init'] = self.lab_data["InitializationHandler"]
        self.h_name['pose'] = self.lab_data["PoseHandler"][0]
        self.h_name['sensor'] = self.lab_data["SensorHandler"][0]
        self.h_name['actuator'] = self.lab_data["ActuatorHandler"][0]
        self.h_name['locomotionCommand'] = self.lab_data["LocomotionCommandHandler"][0]
        self.h_name['motionControl'] = self.robot_data["MotionControlHandler"][0]
        self.h_name['drive'] = self.robot_data["DriveHandler"][0]
    
    def runInitialization(self):
        # We treat initialization handlers separately, because there may be multiple ones
        # NOTE: These will be loaded in the same order as they are listed in the config file

        self.shared_data = {}  # This is for storing things like server connection objects, etc.
        init_num = 1
        self.init_handlers = []
        for handler in self.h_name['init']:
            print "  -> %s" % handler
            # TODO: Is there a more elegant way to do this? This is pretty ugly...
            exec("from %s import initHandler as initHandler%d" % (handler, init_num)) in locals() # WARNING: This assumes our input data is not malicious...
            exec("self.init_handlers.append(initHandler%d(self.project_root, self.project_basename, self.exp_cfg_data, self.robot_data, self.fwd_coordmap, self.rfi, calib=False))" % (init_num)) in locals()
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

        # Now do the rest of them
        for handler in list:
            print "  -> %s" % self.h_name[handler]
            exec("from %s import %sHandler" % (self.h_name[handler], handler)) in locals() # WARNING: This assumes our input data is not malicious...
            if handler == 'pose':
                self.pose_handler = poseHandler(self.shared_data)
            elif handler == 'sensor':
                # Figure out what sensors are enabled, and which are initially true
                self.all_sensors = []
                initial_sensors = []
                for line in self.spec_data['SETTINGS']['Sensors']:
                    sensor, val = line.split(',')
                    if int(val) == 1: 
                        self.all_sensors.append(sensor)
                        if sensor in self.exp_cfg_data['InitialTruths']:
                            initial_sensors.append(sensor)

                self.sensor_handler = sensorHandler(self.shared_data, self.all_sensors, initial_sensors)
            elif handler == 'actuator':
                # Figure out what actuators are enabled
                self.all_actuators = []
                for line in self.spec_data['SETTINGS']['Actions']:
                    act, val = line.split(',')
                    if int(val) == 1: 
                        self.all_actuators.append(act)

                self.actuator_handler = actuatorHandler(self.shared_data)
            elif handler == 'locomotionCommand':
                self.loco_handler = locomotionCommandHandler(self.shared_data)
            elif handler == 'drive':
                self.drive_handler = driveHandler(self.shared_data, self.loco_handler)
            elif handler == 'motionControl':
                self.motion_handler = motionControlHandler(self.shared_data, self.pose_handler, self.drive_handler, self.rfi, self.fwd_coordmap)
