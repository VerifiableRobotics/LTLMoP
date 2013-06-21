#!/usr/bin/env python


""" ================================================
    hsubConfigObjects.py - Defines necessary config objects for 
    handlerSubsystem.py
    ================================================

    This module defines objects for handler system.
    It also provides interface that deal with handler and config files
"""

import os, sys, re
import fileMethods
import inspect,types
from numpy import *
from copy import deepcopy
import ast
import json
import traceback
import globalConfig, logging
import importlib
import handlers.handlerTemplates as ht


###################################################
# Define individual objects for handler subsystem #
###################################################
class MethodParameterConfig(object):
    """
    An argument to a handler method
    Contains name, description, type, value, and default as determined
    by performing introspection on the individual handler modules
    """

    def __init__(self, name="", para_type="", desc="", default=None, max_val=None, min_val=None, value=None):
        self.name = name                # name of the parameter
        self.para_type = para_type      # type of the parameter
        self.desc = desc                # description of the parameter
        self.default = default          # the default values of the parameter
        self.max_val = max_val          # the max value allowed for the parameter
        self.min_val = min_val          # the min value allowed for the parameter
        self.value = value              # the value user set for the parameter
    
    def __repr__(self):
        """
        Overwrite string representation function
        """
        strRepr = ""
        # Only show the atrributes we are interested in
        keyList = ['name','para_type','default','max_val','min_val','value']
        for key in keyList:
            strRepr = strRepr + ("{0:12}{1}\n".format("<"+key+">:",getattr(self,key,'NOT DEFINED')))
        reprString = "\n -- Method Parameter <{0}> -- \n".format(self.name) + \
                    strRepr + " -- End of Method Parameter <{0}> -- \n".format(self.name) 
        return reprString

    def setValue(self,value):
        """
        This function makes sure all parameter are set according to the desired type
        """

        # Change None para_type to empty string to avoid None.lower()
        if self.para_type == None: self.para_type = ""

        if self.para_type.lower() in ['float','double']:
            try:
                self.value = float(value)
            except ValueError:
                logging.error("Invalid float value: {0} for parameter {1}".format(value,self.name))
        elif self.para_type.lower() in ['int','integer']:
            try:
                self.value = int(value)
            except ValueError:
                logging.error("Invalid int value: {0} for parameter {1}".format(value,self.name))
        elif self.para_type.lower() == 'bool' or self.para_type.lower() == 'boolean':
            if str(value).lower() in ['1','true','t']:
                self.value = True
            elif str(value).lower() in ['0','false','f']:
                self.value = False
        elif self.para_type.lower() == 'region':
            try:
                self.value = value.strip("'\"")
            except ValueError:
                logging.error("Invalid region value: {0} for parameter {1}".format(value,self.name))
        elif self.para_type.lower() in ['str','string']:
            try:
                self.value = str(value).strip('\"\'') 
            except ValueError:
                logging.error("Invalid string value: {0} for parameter {1}".format(value,self.name))
        elif self.para_type.lower() == 'choice':
            try:
                self.value = ast.literal_eval(value)
            except ValueError:
                logging.error("Invalid choice value: {0} for parameter {1}".format(value,self.name))
        elif self.para_type.lower() == 'multichoice':
            try:
                self.value = ast.literal_eval(value)
            except ValueError:
                logging.error("Invalid multichoice value: {0} for parameter {1}".format(value,self.name))
        else:
            logging.error("Cannot set the value of parameter {0}, because its type {1} cannot be recognized.".format(self.name,self.para_type))

    def getValue(self):
        return self.value

    def resetValue(self):
        # Reset the parameter value to its default value.
        # If the default value is not define, then the value is set to None
        if self.default == None:
            self.value = None
        else:
            self.setValue(self.default)

    def fromString(self, para_string, method_config):
        """
        Create a MethodParameterConfig from a line of comments in the method object

        para_string : a line of comments in the method object specifies the properties of the parameter
        method_config: a HandlerMethodConfig object where the parameter exists
        """
        # Regular expressions to help us out
        argRE = re.compile('(?P<argName>\w+)(\s*\((?P<type>\w+)\s*\))(\s*:\s*)(?P<description>[^\(]+)(\s*\((?P<range>.+)\s*\))?',re.IGNORECASE)
        settingRE = re.compile(r"""(?x)
                                   (?P<key>\w+)
                                   \s*=\s*
                                   (?P<val>
                                       "[^"]*" | # double-quoted string
                                       \[[^\]]*\] | # array
                                       [^,\s]*    # other
                                    )""")

        # update the parameter object
        m = argRE.search(para_string)
        self.para_type = m.group('type')
        self.desc = m.group('description')
        if m.group('range') is not None:
            try:
                for k, v in settingRE.findall(m.group('range')):
                    if k.lower() not in ['default', 'min_val', 'max_val', 'options', 'min', 'max']:
                        logging.warning('Unrecognized setting name "{0}" for parameter "{1}" of method "{2}"'.format(k.lower(), m.group('argName'),method_config.name))
                        continue

                    setattr(self, k.lower(), json.loads(v.lower()))
            except ValueError:
                # LOG AN ERROR HERE
                logging.warning('Could not parse settings for parameter "{0}" of method "{1}"'.format(m.group('argName'),method_config.name))

            self.resetValue()


class HandlerMethodConfig(object):
    """
    A method object
    Each object represents one method of a given handler
    """
    def __init__(self, name="", handler=None, comment="", para=None, omit_para=None):
        self.name = name            # name of the method
        self.handler = handler      # which handler the method belongs to
        self.comment = comment      # comment of the method
        self.para = para            # list of method parameter config of this method
        self.omit_para = omit_para  # list of parameter names that are omitted

        # To avoid recursive setting
        if self.para is None:
            self.para = []
        # To avoid recursive setting
        if self.omit_para is None:
            self.omit_para = []

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
            elif key == 'handler':
                strRepr = strRepr + ("{0:12}{1}\n".format("<"+key+">:",getattr(getattr(self,key,'NOT DEFINED'),'name','NOT DEFINED')))
            else:
                strRepr = strRepr + ("{0:12}{1}\n".format("<"+key+">:",getattr(self,key,'NOT DEFINED')))
        reprString = "\n --Handler Method <{0}> -- \n".format(self.name) + \
                    strRepr + " -- End of Handler Method <{0}> -- \n".format(self.name) 
        return reprString


    def getParaByName(self, name):
        # get the parameter object with given name
        for p in self.para:
            if p.name == name:
                return p
        logging.error("Could not find parameter of name '{0}' in method '{1}'".format(name, self.name))
        return MethodParameterConfig()

    def fromMethod(self, method, handler_config):
        """
        Create a HandlerMethodConfig from the python method object

        method: python method object
        handler_config: instance of HandlerConfig where this HandlerMethodConfig locates
        """
        # Regular expressions to help us out
        argRE = re.compile('(?P<argName>\w+)(\s*\((?P<type>\w+)\s*\))(\s*:\s*)(?P<description>[^\(]+)(\s*\((?P<range>.+)\s*\))?',re.IGNORECASE)

        self.name = method.__name__
        self.handler = handler_config

        # parsing parameters of the method
        for para_name in inspect.getargspec(method)[0]:
            # create a parameter object if the parameter is not ignorable
            if para_name not in self.handler.ignore_parameter:
                para_config = MethodParameterConfig(para_name)
                self.para.append(para_config)
            else:
                self.omit_para.append(para_name)

        # parse the description of the function
        doc = inspect.getdoc(method)
        if doc is not None:
            for line in doc.split('\n'):

                # If it is an empty line, ignore it
                if re.search('^(\s*)$',line):
                    continue

                # If there is a newline at the end, remove it
                if re.search('\n$',line):
                    line = re.sub('\n$','',line)

                # If the line defines an argument variable
                if argRE.search(line):
                    m = argRE.search(line)
                    for para_config in self.para:
                        # match the definition of a parameter and a parameter object with their names
                        if para_config.name.lower() == m.group('argName').strip().lower():
                            # update the parameter config
                            para_config.fromString(line,self)

                # If the line comments the function
                else:
                    self.comment += line + "\n"
        self.comment = self.comment.strip()

        # remove the parameter that has no definition
        argToRemove = []
        for para_config in self.para:
            if para_config.desc == "":
                argToRemove.append(para_config)
        map(self.para.remove,argToRemove)

class HandlerConfig(object):
    """
    A handler object!
    """
    def __init__(self, name="", h_type=None, shared="", methods=None, robot_type=""):
        self.name = name                # name of the handler
        self.h_type = h_type            # type of the handler e.g. motionControl or drive
        self.shared = shared            # whether the handler is in the shared folder ("y") or not ("n")
        self.methods = methods          # list of method objects in this handler
        # To avoid recursive setting
        if self.methods is None:
            self.methods = []
        self.robot_type = robot_type    # type of the robot using this handler for robot specific handlers
        self.ignore_parameter = ['self','initial','proj','shared_data','actuatorVal']
                                        # list of name of parameter that should be ignored where parse the handler methods

    def __repr__(self):
        """
        Overwrite string presentation function
        """
        strRepr = ""
        # Only show the atrributes we are interested in
        keyList = ['name','h_type','methods','robot_type']
        for key in keyList:
            if key == 'methods':
                strRepr = strRepr + ("{0:13}{1}\n".format("<"+key+">:",','.join([p.name for p in getattr(self,key,[])])))
            else:
                strRepr = strRepr + ("{0:13}{1}\n".format("<"+key+">:",getattr(self,key,'NOT DEFINED')))
        reprString = "\n --Handler <{0}> -- \n".format(self.name) + \
                    strRepr + " -- End of Handler <{0}> -- \n".format(self.name) 
        return reprString

    def getMethodByName(self, name):
        for m in self.methods:
            if m.name == name:
                return m
        logging.error("Could not find method of name '{0}' in handler '{1}'".format(name, self.name))
        return HandlerMethodConfig()

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

    def parseHandler(self,handler_module_path,onlyLoadInit=False):
        """
        Load method info (name,arg...) in the given handler file
        If onlyLoadInit is True, only the info of __init__ method will be loaded
        If over_write_h_type is given, then over write the handler type with it

        returns a handler object or None if fail to load the given handler file
        """

        # add lib to the module name if it is not there already
        if not handler_module_path.startswith('lib.'): handler_module_path = 'lib.' + handler_module_path
        handler_module_name = handler_module_path.rpartition('.')[2]
        # Try to load the handler file
        logging.debug("Inspecting handler: {}".format(handler_module_name))
        try:
            handler_module = importlib.import_module(handler_module_path)
        except Exception as e:
            logging.warning("Failed to import handler {0} : {1}".format(handler_module_name,e))
            if not isinstance(e, ImportError):
                logging.debug(traceback.format_exc())
            raise ImportError


        # Find the class object that specifies the handler
        handler_class = inspect.getmembers(handler_module, lambda c: inspect.isclass(c) and \
                                                                   c.__module__ == handler_module_path and \
                                                                   ht.Handler in inspect.getmro(c))

        # Raise error if there are no handler_class found in the handler file
        if len(handler_class) < 1:
            logging.warning("No handler class found in file {}. Abort importing.".format(handler_module_name))
            raise ImportError
        # Warn if there are multiple handler classes in one handler file
        if len(handler_class) > 1:
            logging.warning("Multiple handler classes found in file {}. Randomly choose one to import.".format(handler_module_name))
        handler_class = handler_class[0][1]
        if handler_class.__name__.lower() != handler_module_name.lower():
            logging.warning("File name: {0} mismatch with class name: {1}.".format(handler_class.__name__,handler_module_name))

        # update the handler name and type info
        # handler name is the name of the file
        # handler type is the corresponding handler object defined in handlerTemplates.py
        self.name = handler_module_name
        self.h_type = inspect.getmro(handler_class)[1] # direct parent

        # get all methods in this handler
        handler_methods = inspect.getmembers(handler_class,inspect.ismethod)
        # parse each method into method object
        for method_name,method in handler_methods:
            # only parse the method not start with underscore (exclude __inti__)
            # only parse the __init__ method if required
            if ((not onlyLoadInit and (not str(method_name).startswith('_')) or str(method_name)=='__init__') ):
                method_config = HandlerMethodConfig(name=method_name)
                method_config.fromMethod(method,self)
                # add this method into the method list of the handler
                self.methods.append(method_config)

class RobotConfig(object):
    """
    A Robot config object
    """
    def __init__(self, r_name = "", r_type = "", handlers = None):
        self.name = r_name              # name of the robot
        self.r_type = r_type            # type of the robot
        self.handlers = handlers        # dictionary of handler object for this robot
        # To avoid recursive setting
        if self.handlers is None:
            self.handlers = {}
        self.calibration_matrix = None  # 3x3 matrix for converting coordinates, stored as lab->map

    def __repr__(self):
        """
        Overwrite string representation function
        """
        strRepr = ""
        # Only show the atrributes we are interested in
        keyList = ['name','r_type','handlers']
        for key in keyList:
            if key == 'handlers':
                handlerDict = getattr(self,key,{})
                strRepr = strRepr + ("{0}{1}\n".format("<"+key+">:\n", \
                '\n'.join(["{0:13}{1:23}{2}".format('',handlerType+':',getattr(handlerDict[handlerType],'name','None')) for handlerType in handlerDict.keys()])))
            else:
                strRepr = strRepr + ("{0:13}{1}\n".format("<"+key+">:",getattr(self,key,'NOT DEFINED')))
        reprString = "\n --Robot <{0}> -- \n".format(self.name) + \
                    strRepr + " -- End of Robot <{0}> -- \n".format(self.name) 
        return reprString

class ExperimentConfig(object):
    """
    A config file object!
    """
    def __init__(self, name="", robots = None, prop_mapping = {}, initial_truths = None , main_robot = "", region_tags = {}, file_name = ""):
        self.name = name                    # name of the config file
        self.robots = robots                # list of robot object used in this config file
        self.prop_mapping = prop_mapping    # dictionary for storing the propositions mapping
        self.initial_truths = initial_truths# list of initially true propoisitions
        self.main_robot = main_robot        # name of robot for moving in this config
        self.region_tags = region_tags      # dictionary mapping tag names to region groups, for quantification
        self.file_name = file_name          # full path filename of the config
        
        # To avoid recursive setting
        if self.robots is None:
            self.robots = []
        # To avoid recursive setting
        if self.initial_truths is None:
            self.initial_truths = []

    def __repr__(self):
        """
        Overwrite string representation function
        """
        strRepr = ""
        # Only show the atrributes we are interested in
        keyList = ['name','robots','main_robot','initial_truths','prop_mapping','file_name']
        for key in keyList:
            if key in ['robots','initial_truths']:
                strRepr = strRepr + ("{0:18}{1}\n".format("<"+key+">:",','.join([getattr(p,'name',p) for p in getattr(self,key,[])])))
            elif key == 'prop_mapping':
                prop_mapping = getattr(self,key,{})
                strRepr = strRepr + "{0}{1}\n".format("<"+key+">:\n", \
                '\n'.join(["{0:18}{1}".format('',prop+' = ' + prop_mapping[prop]) for prop in prop_mapping.keys()]))
            elif key == 'main_robot':
                strRepr = strRepr + ("{0:18}{1}\n".format("<"+key+">:",getattr(getattr(self,key,'NOT DEFINED'),'name','NOT DEFINED')))
            else:
                strRepr = strRepr + ("{0:18}{1}\n".format("<"+key+">:",getattr(self,key,'NOT DEFINED')))
        reprString = "\n --Config <{0}> -- \n".format(self.name) + \
                    strRepr + " -- End of Config <{0}> -- \n".format(self.name) 
        return reprString
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
        configObj = ExperimentConfig()
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
        return RobotConfig()

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
        file_name = self.name.replace(' ','_')

        # Add extension to the name
        file_name = file_name+'.config'

        # Add the path to the file name
        file_name = os.path.join(os.path.dirname(self.file_name),file_name)
        self.file_name = file_name

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

            data[header]['CalibrationMatrix'] = repr(robot.calibration_matrix)
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


        comments = {"FILE_HEADER": "This is a configuration definition file in folder \"%s\".\n" % os.path.dirname(self.file_name)+
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

        fileMethods.writeToFile(file_name, data, comments)
        return True

if __name__ == '__main__':
#    m = HandlerMethodConfig()
#    print m
#    m = MethodParameterConfig('Jim')
#    print m
    m = HandlerConfig()
    try:
        m.parseHandler('handlers.basicSim.basicSimLocomotionCommand')
    except ImportError:
        logging.warning('Cannot import file {}'.format('init'))
    print m
    for method in m.methods:
        print method
