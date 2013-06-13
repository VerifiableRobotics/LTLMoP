#!/usr/bin/env python


""" ================================================
    hsubConfigObjects.py - Defines necessary config objects for 
    handlerSubsystem.py
    ================================================

    This module Defines objects for handler system.
    It also provides interface that deal with handler and config files
"""

import os, sys, re
import fileMethods, regions
import inspect,types
from numpy import *
from copy import deepcopy
import project
import ast
import json
import traceback
import globalConfig, logging


###################################################
# Define individual objects for handler subsystem #
###################################################
class MethodParameterConfig(object):
    """
    An argument to a handler method
    Contains name, description, type, value, and default as determined
    by performing introspection on the individual handler modules
    """

    def __init__(self,para_name=""):
        self.name = para_name   # name of the parameter
        self.type = ""      # type of the parameter
        self.des = ""       # description of the parameter
        self.default = None # the default values of the parameter
        self.max = None     # the max value allowed for the parameter
        self.min = None     # the min value allowed for the parameter
        self.value = None   # the value user set for the parameter
    
    def __repr__(self):
        """
        Overwrite string representation function
        """
        strRepr = ""
        # Only show the atrributes we are interested in
        keyList = ['name','type','default','max','min','value']
        for key in keyList:
            strRepr = strRepr + ("{0:12}{1}\n".format("<"+key+">:",getattr(self,key,'NOT DEFINED')))
        reprString = " -- Method Parameter <{0}> -- \n".format(self.name) + \
                    strRepr + " -- End of Method Parameter <{0}> -- \n".format(self.name) 
        return reprString

    def setValue(self,value):
        """
        This function makes sure all parameter are set according to the desired type
        """

        # Change None type to empty string to avoid None.lower()
        if self.type == None: self.type = ""

        if self.type.lower() in ['float','double']:
            try:
                self.value = float(value)
            except ValueError:
                logging.error("Invalid float value: {0} for parameter {1}".format(value,self.name))
        elif self.type.lower() in ['int','integer']:
            try:
                self.value = int(value)
            except ValueError:
                logging.error("Invalid int value: {0} for parameter {1}".format(value,self.name))
        elif self.type.lower() == 'bool' or self.type.lower() == 'boolean':
            if str(value).lower() in ['1','true','t']:
                self.value = True
            elif str(value).lower() in ['0','false','f']:
                self.value = False
        elif self.type.lower() == 'region':
            try:
                self.value = ast.literal_eval(value)
            except ValueError:
                logging.error("Invalid region value: {0} for parameter {1}".format(value,self.name))
        elif self.type.lower() in ['str','string']:
            try:
                self.value = str(value).strip('\"\'') 
            except ValueError:
                logging.error("Invalid string value: {0} for parameter {1}".format(value,self.name))
        elif self.type.lower() == 'choice':
            try:
                self.value = ast.literal_eval(value)
            except ValueError:
                logging.error("Invalid choice value: {0} for parameter {1}".format(value,self.name))
        elif self.type.lower() == 'multichoice':
            try:
                self.value = ast.literal_eval(value)
            except ValueError:
                logging.error("Invalid multichoice value: {0} for parameter {1}".format(value,self.name))
        else:
            logging.error("Cannot set the value of parameter {0}, because its type {1} cannot be recognized.".format(self.name,self.type))

    def getValue(self):
        return self.value

    def resetValue(self):
        # Reset the parameter value to its default value.
        # If the default value is not define, then the value is set to None
        if self.default == None:
            self.value = None
        else:
            self.setValue(self.default)


class HandlerMethodConfig(object):
    """
    A method object
    Each object represents one method of a given handler
    """
    def __init__(self):
        self.name = ""        # name of the method
        self.handler = None     # which handler the method belongs to
        self.comment = ""       # comment of the method
        self.para = []          # list of method parameter config of this method
        self.omitPara = []      # list of parameter names that are omitted

    def __repr__(self):
        """
        Overwrite string representation function
        """
        strRepr = ""
        # Only show the atrributes we are interested in
        keyList = ['name','handler','para']
        for key in keyList:
            if key == 'para':
                strRepr = strRepr + ("{0:12}{1}\n".format("<"+key+">:",','.join([p.name for p in getattr(self,key,[])])))
            else:
                strRepr = strRepr + ("{0:12}{1}\n".format("<"+key+">:",getattr(self,key,'NOT DEFINED')))
        reprString = " -- Handler Method <{0}> -- \n".format(self.name) + \
                    strRepr + " -- End of Handler <{0}> -- \n".format(self.name) 
        return reprString


    def getParaByName(self, name):
        # get the parameter object with given name
        for p in self.para:
            if p.name == name:
                return p
        logging.error("Could not find parameter of name '{0}' in method '{1}'".format(name, self.name))
        return ParameterObject()

class HandlerObject:
    """
    A handler object!
    """
    def __init__(self):
        self.name = "" # name of the handler
        self.type = "" # type of the handler e.g. motionControl or drive
        self.shared = ""    # whether the handler is in the shared folder ("y") or not ("n")
        self.methods = []   # list of method objects in this handler
        self.robotType = "" # type of the robot using this handler for robot specific handlers

    def __repr__(self):
        """
        Overwrite string presentation function
        """
        strRepr = []
        # Get all attribute names and values
        for key,val in self.__dict__.iteritems():
            # Only show if the value is not None or empty
            if val: strRepr.append("{0}:{1}".format(key,val))
        
        # if all attributes have values of None or empty
        if not strRepr: 
            reprString = "All attributes have values of None or empty."
        else:
            reprString = "\n".join(strRepr)
        
        return "Handler Object -- \n" + reprString + "\n"

    def getMethodByName(self, name):
        for m in self.methods:
            if m.name == name:
                return m
        logging.error("Could not find method of name '{0}' in handler '{1}'".format(name, self.name))
        return MethodObject()

    def toString(self,forsave = True):
        """
        Return the string representation of the handler object

        forsave is True then the string is for saving the config file
        False is for initiate this handler during execution
        """
        # prepare the input for initiation
        initMethodObj = None
        for methodObj in self.methods:
            if methodObj.name == '__init__':
                initMethodObj = methodObj
                break

        method_input = []
        for paraObj in initMethodObj.para:
            if paraObj.type.lower() in ['str', 'string', 'region']:
                if paraObj.value is not None:
                    method_input.append('%s=%s'%(paraObj.name,'\"'+paraObj.value+'\"'))
            else:
                method_input.append('%s=%s'%(paraObj.name,str(paraObj.value)))
            # change the deliminator of list into ";"
            if forsave and paraObj.type.startswith('listof'):
                method_input[-1]=method_input[-1].replace(',',';')

        if not forsave:
            for para_name in initMethodObj.omitPara:
                if para_name == 'initial':
                    method_input.append('%s=%s'%(para_name,'True'))
                elif para_name == 'proj':
                    method_input.append('%s=%s'%(para_name,'self.proj'))
                elif para_name == 'shared_data':
                    method_input.append('%s=%s'%(para_name,'self.proj.shared_data'))

        method_input = '('+','.join(method_input)+')'

        if not forsave:
            string = method_input
        else:
            string = self.name+method_input
        return string

    def fullPath(self,robotName,configObj):
        """ Return the full path for this handler object for importing"""
        if self.getType() in ['init','sensor','actuator','locomotionCommand']:
            for robotObj in configObj.robots:
                if robotObj.name == robotName:
                    robotFolder = robotObj.type
                    break
            if robotName=='share':
                fileName = '.'.join(['handlers.share',self.name])
            else:
                fileName = '.'.join(['handlers.robots',robotFolder,self.name])
        else:
            fileName = '.'.join(['handlers',self.getType(),self.name])
        return fileName

    def getType(self):
        return self.type
    def setType(self,h_type):
        self.type=h_type
        return self

class RobotObject:
    """
    A Robot object
    """
    def __init__(self,r_name="" ,r_type="",driveH=None,initH=None,locoH=None,motionH=None,poseH=None,sensorH=None,actuatorH=None):
        self.name = r_name  # name of the robot
        self.type = r_type  # type of the robot
        self.handlers = {'drive':driveH, 'init':initH, 'locomotionCommand':locoH, 'motionControl':motionH, 'pose':poseH, 'sensor':sensorH,'actuator':actuatorH} # dictionary of handler object for this robot
        self.calibrationMatrix = None # 3x3 matrix for converting coordinates, stored as lab->map

    def __repr__(self):
        """
        Overwrite string representation function
        """
        strRepr = []
        # Get all attribute names and values
        for key,val in self.__dict__.iteritems():
            # Only show if the value is not None or empty
            if key == "handlers":
                for h_type,h_obj in self.handlers.iteritems():
                    if h_obj:
                        strRepr.append("{0}:{1}".format(key,val)) 
                        break

            elif val: strRepr.append("{0}:{1}".format(key,val))
        
        # if all attributes have values of None or empty
        if not strRepr: 
            reprString = "All attributes have values of None or empty."
        else:
            reprString = "\n".join(strRepr)
        
        return "Robot Object -- \n" + reprString + "\n"

class ConfigObject:
    """
    A config file object!
    """
    def __init__(self):
        self.name = "" # name of the config file
        self.robots = []    # list of robot object used in this config file
        self.prop_mapping = {}  # dictionary for storing the propositions mapping
        self.initial_truths = [] # list of initially true propoisitions
        self.main_robot = "" # name of robot for moving in this config
        self.region_tags = {} # dictionary mapping tag names to region groups, for quantification
        self.fileName = ""  # full path filename of the config

    def __repr__(self):
        """
        Overwrite string representation function
        """
        strRepr = []
        # Get all attribute names and values
        for key,val in self.__dict__.iteritems():
            # only show if the value is not none or empty
            if val: strrepr.append("{0}:{1}".format(key,val))
        
        # if all attributes have values of None or empty
        if not strRepr: 
            reprString = "All attributes have values of None or empty."
        else:
            reprString = "\n".join(strRepr)
        
        return "Config Object -- \n" + reprString + "\n"
    
    def checkComplete(self):
        """
        This function checks if all of the attributes of the config object are completed by comparing them
        with the ones of a default config object
        Return a list of attriute names that are not completed
        Return an empty list if all attributes are completed
        """

        incomplete_attr = []
        # First, let's create a default config object
        configObj = ConfigObject()
        # Then let's compare all attributes
        for key,val in self.__dict__.iteritems():
            if val == getattr(configObj,key):
                incomplete_attr.append(key)
                
        return incomplete_attr

    def getRobotByName(self, name):
        for r in self.robots:
            if r.name == name:
                return r
        logging.error("Could not find robot of name '{0}' in config '{1}'.".format(name, self.name))
        return RobotObject()

    def saveConfig(self):
        """
        Save the config object. 
        Return True for successfully saved, False for not
        """
        # Check if the config object is a complete one, if not then throw warning.
        incomplete_attr = self.checkComplete()
        if incomplete_attr:
            logging.warning("Attribute {0} of Config {1} is not complete. Save those attributes with default value.".format(incomplete_attr,self.name))

        # the file name is default to be the config name with underscore
        if 'name' in incomplete_attr:
            self.name = 'Untitled configuration'
        fileName = self.name.replace(' ','_')

        # Add extension to the name
        fileName = fileName+'.config'

        # Add the path to the file name
        fileName = os.path.join(os.path.dirname(self.fileName),fileName)
        self.fileName = fileName

        data = {'General Config':{'Name':self.name}}

        # proposition mapping
        sensorMappingList = []
        actuatorMappingList = []
        for prop, fun in self.prop_mapping.iteritems():
            if 'sensor' in fun.lower():
                sensorMapping = prop + ' = ' + fun
                sensorMappingList.append(sensorMapping)
            elif 'actuator' in fun.lower():
                actuatorMapping = prop + ' = ' + fun
                actuatorMappingList.append(actuatorMapping)
            else:
                logging.warning("Cannot recognize prop mapping: {}".format(prop+" = "+fun))

        data['General Config']['Sensor_Proposition_Mapping'] = sensorMappingList
        data['General Config']['Actuator_Proposition_Mapping'] = actuatorMappingList
        data['General Config']['Main_Robot'] = self.main_robot
        data['General Config']['Initial_Truths'] = self.initial_truths
        data['General Config']['Region_Tags'] = json.dumps(self.region_tags)

        for i,robot in enumerate(self.robots):
            header = 'Robot'+str(i+1)+' Config'
            data[header]={}
            data[header]['RobotName'] = robot.name
            data[header]['Type'] = robot.type

            data[header]['CalibrationMatrix'] = repr(robot.calibrationMatrix)
            # TODO: change to string function
            try:
                data[header]['InitHandler'] = robot.handlers['init'].toString()
                data[header]['PoseHandler'] = robot.handlers['pose'].toString()
                data[header]['MotionControlHandler'] = robot.handlers['motionControl'].toString()
                data[header]['DriveHandler'] = robot.handlers['drive'].toString()
                data[header]['LocomotionCommandHandler'] = robot.handlers['locomotionCommand'].toString()
                data[header]['SensorHandler'] = robot.handlers['sensor'].toString()
                data[header]['ActuatorHandler'] = robot.handlers['actuator'].toString()
            except AttributeError:
                logging.warning("Cannot save all handlers for robot {}. Please make sure they are all successfully loaded. Aborting saving.".format(robot.name))
                return False


        comments = {"FILE_HEADER": "This is a configuration definition file in folder \"%s\".\n" % os.path.dirname(self.fileName)+
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
                    "CalibrationMatrix": "3x3 matrix for converting coordinates, stored as lab->map",
                    "Actuator_Proposition_Mapping": 'Mapping between actuator propositions and actuator handler functions',
                    "Sensor_Proposition_Mapping": "Mapping between sensor propositions and sensor handler functions",
                    "Name": 'Configuration name',
                    "Main_Robot":'The name of the robot used for moving in this config',
                    "Initial_Truths": "Initially true propositions",
                    "Region_Tags": "Mapping from tag names to region groups, for quantification"}

        fileMethods.writeToFile(fileName, data, comments)
        return True
