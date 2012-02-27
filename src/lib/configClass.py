#!/usr/bin/env python

""" ================================================
    configClass.py - Object from a .config file
    ================================================
    
    This module exposes an object that allows for loading/editing/saving the data 
    in a given .config file
"""

import os, sys
import fileMethods, regions
from numpy import *

class RobotObject:
    """
    A Robot object
    """
    def __init__(self,r_id=None,r_name=None,r_type=None,driveH=None,initH=None,locoH=None,motionH=None,poseH=None,sensorH=None,actuatorH=None):
        self.id = r_id
        self.name = r_name
        self.type = r_type
        self.robotSpeHandler = ['init','locomotionCommand','sensor','actuator']
        self.handlers = {'drive':driveH, 'init':initH, 'locomotionCommand':locoH, 'motionControl':motionH, 'pose':poseH, 'sensor':sensorH,'actuator':actuatorH}

class configClass:
    """
    A config object.
    """

    def __init__(self,proj):
        self.proj = proj
        self.silent = False

        self.robots =[]
        self.name = ''
        self.propMapping = {}
        self.initial_truths = []
        pass

    def setSilent(self, silent):
        self.silent = silent

    def loadConfigFile(self,fileName):
        
        # Add extension to the name if there isn't one. 
        if not fileName.endswith('.config'):
            fileName = fileName+'.config'  
     
        if not self.silent: print "Loading config file %s..." % fileName
        try:
            # First try path relative to project path
            self.config_data = fileMethods.readFromFile(os.path.join(os.path.join(self.proj.project_root,'configs'), fileName))   
        except IOError: 
            if not self.silent: print "ERROR: Cannot find config file %s" % fileName        
            return
    
        self.name = self.config_data['General Config']['Name'][0]

        # parse the string for prop mapping
        for propMapping in self.config_data['General Config']['Proposition_Mapping']:
            try:
                prop,fun = [s.strip() for s in propMapping.split('=',1)]
            except IOError: 
                if not self.silent: print "ERROR: Wrong proposition mapping -- %s" % propMapping        
                return

            self.propMapping[prop]=fun

        # load robot configs
        for configKey,configValue in self.config_data.iteritems():
            if configKey.startswith('Robot'):
                try:
                    robot = RobotObject(r_id = configKey.replace('Robot','').replace(' Config',''),r_name = configValue['RobotName'][0],r_type = configValue['Type'][0])
                    robot.handlers['init'] = configValue['InitHandler'][0]
                    robot.handlers['pose'] = configValue['PoseHandler'][0]
                    robot.handlers['motionControl'] = configValue['MotionControlHandler'][0]
                    robot.handlers['drive'] = configValue['DriveHandler'][0]
                    robot.handlers['locomotionCommand'] = configValue['LocomotionCommandHandler'][0]
                    robot.handlers['sensor'] = configValue['SensorHandler'][0]
                    robot.handlers['actuator'] = configValue['ActuatorHandler'][0]
                    self.robots.append(robot)
                except IOError: 
                    if not self.silent: print "ERROR: Wrong robot configs -- %s" % configKey        
                    return


    def saveConfigFile(self, fileName=''):
        """
        Write all data out to a file.
        """

        if fileName is '':
            fileName = self.name.replace(' ','_')

        
         # Add extension to the name if there isn't one. 
        if not fileName.endswith('.config'):
            fileName = fileName+'.config'  
        

        data = {'General Config':{'Name':self.name}}
        
        # propMapping
        propMappingList = []
        for prop, fun in self.propMapping.iteritems():
            mapping = prop + ' = ' + fun
            propMappingList.append(mapping)

        data['General Config']['Proposition_Mapping'] = propMappingList
                

        for robot in self.robots:
            header = 'Robot'+robot.id+' Config'
            data[header]={}
            data[header]['RobotName'] = robot.name
            data[header]['Type'] = robot.type
            data[header]['InitHandler'] = robot.handlers['init']
            data[header]['PoseHandler'] = robot.handlers['pose']
            data[header]['MotionControlHandler'] = robot.handlers['motionControl']
            data[header]['DriveHandler'] = robot.handlers['drive']
            data[header]['LocomotionCommandHandler'] = robot.handlers['locomotionCommand']
            data[header]['SensorHandler'] = robot.handlers['sensor']
            data[header]['ActuatorHandler'] = robot.handlers['actuator']


        comments = {"FILE_HEADER": "This is a configuration definition file for the example \"%s\".\n" % self.proj.project_basename +
                    "Format details are described at the beginning of each section below.\n",
                    "PoseHandler": "Input value for robot pose handler, refer to file inside the handlers/pose folder",
                    "DriveHandler": "Input value for robot drive handler, refer to file inside the handlers/drive folder",
                    "MotionControlHandler": "Input value for robot motion control handler, refer to file inside the handlers/motionControl folder",
                    "LocomotionCommandHandler": "Input value for robot locomotion command handler, refer to file inside the handlers/robots/Type folder",
                    "InitHandler": "Input value for robot init handler, refer to the init file inside the handlers/robots/Type folder",
                    "SensorHandler": "Sensor handler file in robots/Type folder",
                    "ActuatorHandler": "Actuator handler file in robots/Type folder",
                    "RobotName": "Robot Name",
                    "Type": "Robot type",
                    "Actuator_Proposition_Mapping": 'Mapping between actuator propositions and actuator handler functions',
                    "Sensor_Proposition_Mapping": "Mapping between sensor propositions and sensor handler functions",
                    "Name": 'Configuration name'}

        fileMethods.writeToFile(os.path.join(self.proj.project_root,'configs/'+fileName), data, comments)


