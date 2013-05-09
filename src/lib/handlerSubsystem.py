#!/usr/bin/env python


""" ================================================
    handlerSubsystem.py - Interface for working with handlers, configs
    ================================================

    This module Defines objects for handler system. It also provides interface that deal with handler and config files
"""


import os, sys, re
import fileMethods, regions
import inspect,types
from numpy import *
from copy import deepcopy
import project
import logging
import ast
import json
import traceback


###################################################
# Define individual objects for handler subsystem #
###################################################
class ParameterObject:
    """
    A parameter object
    Each object represents one parameter of a given method
    """

    def __init__(self,para_name):
        self.name = para_name   # name of the parameter
        self.type = None    # type of the parameter
        self.des = None     # description of the parameter
        self.default = None # the default values of the parameter
        self.max = None     # the max value allowed for the parameter
        self.min = None     # the min value allowed for the parameter
        self.value = None   # the value user set for the parameter

    def setValue(self,value):
        """
        This function makes sure all parameter are set according to the desired type
        """

        if self.type.lower() in ['float','double']:
            try:
                self.value = float(value)
            except ValueError:
                logging.error("Invalid float value: {0}".format(value))
        elif self.type.lower() in ['int','integer']:
            try:
                self.value = int(value)
            except ValueError:
                logging.error("Invalid int value: {0}".format(value))
        elif self.type.lower() == 'bool' or self.type.lower() == 'boolean':
            self.value = value
        elif self.type.lower() == 'region':
            try:
                self.value = ast.literal_eval(value)
            except ValueError:
                logging.error("Invalid region value: {0}".format(value))
        elif self.type.lower() in ['str','string']:
            try:
                self.value = str(value).strip('\"\'') 
            except ValueError:
                logging.error("Invalid string value: {0}".format(value))
        elif self.type.lower() == 'choice':
            try:
                self.value = ast.literal_eval(value)
            except ValueError:
                logging.error("Invalid choice value: {0}".format(value))
        elif self.type.lower() == 'multichoice':
            try:
                self.value = ast.literal_eval(value)
            except ValueError:
                logging.error("Invalid multichoice value: {0}".format(value))
        else:
            logging.error("Invalid parameter value: {0}".format(value))

    def getValue(self):
        return self.value

    def resetValue(self):
        # Reset the parameter value to its default value.
        # If the default value is not define, then the value is set to None
        if self.default == None:
            self.value = None
        else:
            self.setValue(self.default)


class MethodObject:
    """
    A method object
    Each object represents one method of a given handler
    """
    def __init__(self):
        self.name = None        # name of the method
        self.handler = None     # which handler the method belongs to
        self.comment = ''       # comment of the method
        self.para = []          # list of parameter object of this method
        self.omitPara = []      # list of parameter names that are omitted

    def getParaByName(self, name):
        # get the parameter object with given name
        for p in self.para:
            if p.name == name:
                return p
        logging.error("Could not find parameter of name '%s' in method '%s'" % (name, self.name))
        return None

class HandlerObject:
    """
    A handler object!
    """
    def __init__(self):
        self.name = None    # name of the handler
        self.type = None    # type of the handler e.g. motionControl or drive
        self.methods = []   # list of method objects in this handler
        self.robotType = '' # type of the robot using this handler for robot specific handlers

    def getMethodByName(self, name):
        for m in self.methods:
            if m.name == name:
                return m
        logging.error("Could not find method of name '%s' in handler '%s'" % (name, self.name))
        return None

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
    def __init__(self,r_name=None,r_type=None,driveH=None,initH=None,locoH=None,motionH=None,poseH=None,sensorH=None,actuatorH=None):
        self.name = r_name  # name of the robot
        self.type = r_type  # type of the robot
        self.handlers = {'drive':driveH, 'init':initH, 'locomotionCommand':locoH, 'motionControl':motionH, 'pose':poseH, 'sensor':sensorH,'actuator':actuatorH} # dictionary of handler object for this robot
        self.calibrationMatrix = None # 3x3 matrix for converting coordinates, stored as lab->map



class HandlerSubsystem:
    """
    Interface dealing with configuration files and hanlders
    """
    def __init__(self,proj,loggerLevel='error'):

        self.proj = proj
        # Set up loggers for printing error messages

        class ColorLogFormatter(logging.Formatter):
            def __init__(self, *args, **kwds):
                super(ColorLogFormatter, self).__init__(*args, **kwds)
                self.formatter = logging.Formatter(" --> [%(levelname)s] (%(pathname)s, line %(lineno)s): %(message)s")

            def colorize(self, level, string):
                if sys.platform in ['win32', 'cygwin']:
                    # Message with color is not yet supported in Windows
                    return string
                else:
                    colors = {'ERROR': 91, 'WARNING': 93, 'INFO': 94, 'DEBUG': 97}
                    return "\033[{0}m{1}\033[0m".format(colors[level], string)

            def format(self, record):
                precolor = self.formatter.format(record)
                return self.colorize(record.levelname, precolor)
                
        logger = logging.getLogger()
        h = logging.StreamHandler()
        f = ColorLogFormatter()
        h.setFormatter(f)
        logger.addHandler(h)

        if loggerLevel == 'error':
            logger.setLevel(logging.ERROR)
        elif loggerLevel == 'warning':
            logger.setLevel(logging.WARNING)
        elif loggerLevel == 'info':
            logger.setLevel(logging.INFO)
        elif loggerLevel == 'debug':
            logger.setLevel(logging.DEBUG)

        self.handler_dic = None
        self.robots = None
        self.configs = None

        # Create Handler parser
        self.handler_path = os.path.join(self.proj.ltlmop_root,'lib','handlers')
        self.handler_parser = HandlerParser(self.handler_path)

        # Create Handler parser
        self.robot_parser = RobotFileParser(self.handler_path)

        # Create Handler parser
        self.config_path = os.path.join(self.proj.project_root,'configs')
        self.config_parser = ConfigFileParser(self.config_path,self.handler_path,self.proj)

        # For debug
        #print self.configs[0].robots[0].handlers['init:nao'].methods[0].para[1].value
        #print self.robots[0].name
        #print "Done!"

    def loadAllHandlers(self):
        self.handler_parser.loadAllHandlers()
        self.handler_dic = self.handler_parser.handler_dic

    def loadAllRobots(self):
        self.robot_parser.handler_dic = self.handler_dic
        self.robot_parser.loadAllRobots()
        self.robots = self.robot_parser.robots

    def loadAllConfigFiles(self):
        self.config_parser.handler_dic = self.handler_dic
        self.config_parser.loadAllConfigFiles()
        self.configs = self.config_parser.configs

    def getRobotByType(self, t):
        for r in self.robots:
            if r.type == t:
                return r
        logging.error("Could not find robot of type '%s'" % (t))
        return None

    def getHandler(self, htype, hname, rname=None):
        if htype in self.handler_parser.handler_robotSpecific_type:
            if rname is None:
                logging.error("Handler of type '%s' requires a robot type to be specified for lookup" % (htype))
                return None

            for h in self.handler_dic[htype][rname]:
                if h.name == hname:
                    return h
        else:
            for h in self.handler_dic[htype]:
                if h.name == hname:
                    return h

        logging.error("Could not find handler of type '%s' with name '%s'" % (htype, hname))
        return None

    def string2Method(self,method_string):
        """
        Return the method object according to the input string
        """

        if self.handler_dic is None:
            logging.error("Cannot find handler dictionary, please load all handlers first.")
            return

        method_info,para_info = method_string.split('(')
        para_info = para_info.replace(')','')

        items = method_info.split('.')
        methodObj = MethodObject()

        if len(items) == 3:
            robotName = items[0]
            handlerName = items[1]
            methodName = items[2]

            handlerObj = None

            if robotName == 'share':
                # this ia a dummy sensor/actuator
                for h in self.handler_dic['share']:
                    if h.name == handlerName:
                        handlerObj = h
                        break
            else:
                # this is a robot sensor/actuator
                for robotObj in self.robots:
                    if robotObj.name == robotName:
                        if 'sensor' in handlerName.lower():
                            handlerObj = robotObj.handlers['sensor']
                        elif 'actuator' in handlerName.lower():
                            handlerObj = robotObj.handlers['actuator']
                        else:
                            logging.error("Cannot recognize handler {}".format(handlerName)) 
                            return

            if handlerObj is None:
                logging.error("Cannot recognize robot {}".format(robotName)) 
                return

            for method_obj in handlerObj.methods:
                if method_obj.name == methodName:
                    methodObj = method_obj
                    break

            for para_name, para_value in re.findall(r'(?P<key>\w+)\s*=\s*(?P<val>"[^"]*"|\'[^\']*\'|[^,]+)', para_info):
                para_value = para_value.strip("\"\'")
                for paraObj in methodObj.para:
                    if paraObj.name == para_name:
                        paraObj.setValue(para_value)
                        break
        else:
            logging.error("Cannot recognize method {}, please spicify which handler it belongs to.".format(method_info))
            return

        return methodObj


    def method2String(self,methodObj,robotName=''):
        """
        Return the string representation according to the input method object
        """
        if self.handler_dic is None:
            logging.error("ERROR: Cannot find handler dictionary, please load all handler first.")
            return
        if not isinstance(methodObj,MethodObject):
            logging.error("ERROR: Input is not a valid method object!")
            return
        if robotName=='':
            logging.error("ERROR: Needs robot name for method2String")
            return

        handlerName = methodObj.handler.name
        methodName = methodObj.name

        # convert all parameter object into string
        para_list = []
        for paraObj in methodObj.para:
            if paraObj.value is None:
                para_list.append( paraObj.name+'='+str(paraObj.default))
            else:
                if paraObj.type.lower() in ['str', 'string', 'region']:
                    para_list.append( paraObj.name+'=\"'+str(paraObj.value)+'\"')
                else:
                    para_list.append( paraObj.name+'='+str(paraObj.value))

        para_info = ','.join(para_list)

        return '.'.join([robotName,handlerName,methodName])+'('+para_info+')'


    def importHandlers(self,configObj,all_handler_types):
        """
        Figure out which handlers we are going to use, based on the different configurations file settings
        Only one motion/pose/drive/locomotion handler per experiment
        Multiple init/sensor/actuator handlers per experiment, one for each robot (if any)
        Note that the order of loading is important, due to inter-handler dependencies.
        """
        self.h_obj = {'init':{},'pose':{},'locomotionCommand':{},'motionControl':{},'drive':{},'sensor':{},'actuator':{}}
        robots = configObj.robots

        # load all handler objects based on the current configuration
        for robotObj in robots:
            handlers = robotObj.handlers
            for handler_type,handlerObj in handlers.iteritems():
                if handler_type in ['init','sensor','actuator']:
                    self.h_obj[handler_type][robotObj.name] = handlerObj
                elif handler_type in ['pose','motionControl','locomotionCommand','drive']:
                    # only load for main robot
                    if robotObj.name == configObj.main_robot:
                        self.h_obj[handler_type][robotObj.name] = handlerObj
                else:
                    logging.error("ERROR: Cannot recognize handler type {}".format(handler_type))

        # load dummy handlers
        handlerParser = HandlerParser(self.handler_path)
        for handlerObj in handlerParser.loadHandler('share'):
            if 'sensor' in handlerObj.name.lower():
                self.h_obj['sensor']['share'] = handlerObj
            elif 'actuator' in handlerObj.name.lower():
                self.h_obj['actuator']['share'] = handlerObj
        # complain if there is any missing handler
        for handler_type, handlerObj in self.h_obj.iteritems():
            if (handlerObj == {}):
                logging.error("ERROR: Cannot find handler for {}".format(handler_type))


        # initiate all handlers
        for handler_type in all_handler_types:
            for robotName,handlerObj in self.h_obj[handler_type].iteritems():
                # get handler class object for initiating
                fileName = handlerObj.fullPath(robotName,configObj)
                logging.debug(" -> Loading handler:\t%s" % fileName.split('.')[-1])
                try:
                    __import__(fileName)
                except ImportError as import_error:
                    logging.error(" -> Failed to import handler %s : %s" % (fileName.split('.')[-1],import_error))

                handlerModule = sys.modules[fileName]
                allClass = inspect.getmembers(handlerModule,inspect.isclass)
                for classObj in allClass:
                    if classObj[1].__module__ == fileName and not classObj[0].startswith('_'):
                        handlerClass = classObj[1]
                        break

                # initiate the handler
                if handler_type in ['init','sensor','actuator']:
                    self.proj.h_instance[handler_type][robotName] = eval('handlerClass'+handlerObj.toString(False))
                    if handler_type == 'init':
                        self.proj.shared_data.update(self.proj.h_instance[handler_type][robotName].getSharedData())
                else:
                    self.proj.h_instance[handler_type] = eval('handlerClass'+handlerObj.toString(False))

        self.proj.sensor_handler = {}
        self.proj.actuator_handler = {}
        self.proj.sensor_handler['initializing_handler']={}
        self.proj.actuator_handler['initializing_handler']={}

        # prepare the regular expression
        methodRE = re.compile(r"(?P<method_string>(?P<robot>\w+)\.(?P<type>\w+)\.(?P<name>\w+)\((?P<args>[^\)]*)\))")

        # initialize sensor and actuator methods
        # first need to get the method used for sensor and actuator

        for prop in self.proj.enabled_sensors:
            if prop in configObj.prop_mapping:
                method = configObj.prop_mapping[prop]
            else:
                # Default to dummysensor
                logging.warning("WARNING: No mapping given for sensor prop '{}', so using default simulated handler.".format(prop))
                method = "share.dummySensor.buttonPress(button_name='%s')" % prop

            fullExpression = method
            codeList = []
            for m in methodRE.finditer(method):
                method_string = m.group('method_string')
                methodEvalString = ''
                robotName = m.group('robot')
                handlerName = m.group('type')
                methodName = m.group('name')
                para_info = [x.strip() for x in m.group('args').replace(')','').split(',')]

                methodEvalString = 'self.h_instance[%s][%s].%s'%('\'sensor\'','\''+robotName+'\'',self.constructMethodString(robotName,'sensor',methodName,para_info))
                codeList.append(methodEvalString)

                fullExpression = fullExpression.replace(method_string,methodEvalString)

            self.proj.sensor_handler['initializing_handler'][prop] = codeList
            self.proj.sensor_handler[prop]=compile(fullExpression,"<string>","eval")

        for prop in self.proj.enabled_actuators:
            if prop in configObj.prop_mapping:
                method = configObj.prop_mapping[prop]
            else:
                # Default to dummyactuator
                logging.warning("WARNING: No mapping given for actuator prop '{}', so using default simulated handler.".format(prop))
                method = "share.dummyActuator.setActuator(name='%s')" % prop

            fullExpression = method.replace(" and ", " ; ")
            # TODO: Complain about ORs
            codeList = []
            for m in methodRE.finditer(method):
                method_string = m.group('method_string')
                methodEvalString = ''
                robotName = m.group('robot')
                handlerName = m.group('type')
                methodName = m.group('name')
                para_info = [x.strip() for x in m.group('args').replace(')','').split(',')]

                methodEvalString = 'self.h_instance[%s][%s].%s'%('\'actuator\'','\''+robotName+'\'',self.constructMethodString(robotName,'actuator',methodName,para_info))
                codeList.append(methodEvalString)

                fullExpression = fullExpression.replace(method_string,methodEvalString)

            self.proj.actuator_handler['initializing_handler'][prop] = codeList
            self.proj.actuator_handler[prop]=compile(fullExpression,"<string>","exec")

    def constructMethodString(self,robotName,handlerName,methodName,para_info):
        """
        returns the string used to execute the corresponding method
        """
        methods = deepcopy(self.h_obj[handlerName][robotName].methods)
        for methodObj in methods:
            if methodObj.name == methodName:
                method_input = []
                for paraObj in methodObj.para:
                    for para_pair in para_info:
                        if paraObj.name == para_pair.split('=',1)[0]:
                            paraObj.setValue(para_pair.split('=',1)[1])
                            break

                    if paraObj.type.lower() in ['str', 'string', 'region']:
                        method_input.append('%s=%s'%(paraObj.name,'\"'+paraObj.value+'\"'))
                    else:
                        method_input.append('%s=%s'%(paraObj.name,str(paraObj.value)))
                for para_name in methodObj.omitPara:
                    if para_name == 'initial':
                        method_input.append('%s=%s'%(para_name,'initial'))
                    elif para_name == 'proj':
                        method_input.append('%s=%s'%(para_name,'self.proj'))
                    elif para_name == 'shared_data':
                        method_input.append('%s=%s'%(para_name,'self.proj.shared_data'))
                    elif para_name == 'actuatorVal':
                        method_input.append('%s=%s'%(para_name,'new_val'))

                method_input = methodName+'('+','.join(method_input)+')'
        return method_input

class HandlerParser:
    """
    A parser loads all handler information
    """
    def __init__(self,path):
        self.handler_path = path    # handler folder path
        self.handler_dic = {}       # dictionary for all handler information {type of handler:list of handlers of that type}
                                    # for sensor,actuator,init and locomotion handler the value is a dictionary {name of the robot: handler object}
        self.handler_all = {}       # dic of handler objects parsed from handler files. {handlerType.handlerFileName:handler object}
        self.handler_types = ['pose','drive','motionControl','share']       # list of types of handlers, must match with the folder name in lib/handler folder
        self.handler_robotSpecific_type = ['init','locomotionCommand','sensor','actuator']   # list of types of robot specific handlers in each robot folder
        self.ignore_parameter = ['self','initial','proj','shared_data','actuatorVal']     # list of name of parameter that should be ignored where parse the handler methods


    def loadAllHandlers(self):
        """
        Load all handlers in the handler folder
        """

        handlerFolders = os.listdir(self.handler_path)

        # load all robot-independent handlers first
        for handler_type in self.handler_types:
            self.handler_dic[handler_type] = []
            if handler_type not in handlerFolders:
                # Can't find the folder containing the type of handlers
                logging.warning("Cannot find %s handler folder in %s" % (handler_type, self.handler_path))
            else:

                # For the robot independent handlers, we only want to parse the init function to get the parameters information
                # But both simulated sensor and actuator handlers are in the shared folder. All of their function need to be parsed.
                if handler_type == 'share':
                    self.handler_dic[handler_type] = self.loadHandler(handler_type,False)
                else:
                    self.handler_dic[handler_type] = self.loadHandler(handler_type,True)

        # now let's load all robot specific hanlders. Store them using nested dictionary with keys of [handlerType][robotType]
        # we also want to add the simulated sensor and actuator handler into the sensor and actuator list even they are robot independent handlers
        self.handler_dic['sensor'] = {'share': [handlerObj.setType('sensor') for handlerObj in self.handler_dic['share'] if 'sensor' in handlerObj.name.lower()]}
        self.handler_dic['actuator'] = {'share': [handlerObj.setType('actuator') for handlerObj in self.handler_dic['share'] if 'actuator' in handlerObj.name.lower()]}
        self.handler_dic['init'] = {}
        self.handler_dic['locomotionCommand'] = {}
        if 'robots' not in handlerFolders:
            # All robot specific handlers should be stored in the robtos folder
            logging.warning("Cannot find robot folder in %s" % (self.handler_path))
        else:
            robotFolderList = os.listdir(os.path.join(self.handler_path,'robots'))
            for robotFolder in robotFolderList:
                # ignore item that is not a folder
                if '.' not in robotFolder:

                    folderName = '.'.join(['robots',robotFolder])

                    fileList = os.listdir(os.path.join(self.handler_path,'robots',robotFolder))
                    for fileName in fileList:
                        for handler_type in self.handler_robotSpecific_type:
                            # ignore non .py file, or file starts with underscore. match the handler type with the handler file name
                            if fileName.endswith('py') and (not fileName.startswith('_')) and handler_type.lower() in fileName.lower():
                                # prepare the string for import this handler file
                                h_file = '.'.join(['handlers','robots',robotFolder,fileName.split('.')[0]])

                                # only load the init function when parsing the init handler and loco handler
                                if handler_type in ['init','locomotionCommand']:
                                    onlyLoadInit = True
                                else:
                                    onlyLoadInit = False
                                # The handler object is stored as a list even there is only one item in the list
                                self.handler_dic[handler_type][robotFolder] = [self.parseHandlers(h_file,handler_type,onlyLoadInit)]

    def loadHandler(self,folder,onlyLoadInit=False):
        """
        Load all handler files within the given folder
        If onlyLoadInit is True, only the info of __init__ method will be loaded
        If over_write_h_type is given, then over write the handler type with it

        return a list of handler objects
        """
        # the list of handler objects that will be returned
        handlerList = []

        path = os.path.join(self.handler_path,folder)
        handlerFileList = os.listdir(path)

        for h_file in handlerFileList:
            # ignore the file starts with underscore or not a .py file
            if h_file.endswith('.py') and not h_file.startswith('_'):
                fileName = '.'.join(['handlers',folder,h_file.split('.')[0]])
                # change the type of handlers if it is robot dependent 
                over_write_h_type=None
                for handler_type in self.handler_robotSpecific_type:
                    if handler_type.lower() in h_file.lower():
                        # this is a robot dependent handler or a handler in the share folder
                        over_write_h_type = handler_type.lower()

                h_obj = self.parseHandlers(fileName,folder,onlyLoadInit,over_write_h_type)
                # update the dictionary with all handler objects
                if over_write_h_type:
                    handler_dic_key = '.'.join([folder,over_write_h_type,h_file.split('.')[0]])
                else:
                    handler_dic_key = '.'.join([folder,h_file.split('.')[0]])
                self.handler_all[handler_dic_key] = h_obj
                if h_obj is not None:
                    # only append the handler in the list if it is correctly imported
                    handlerList.append(h_obj)

        return handlerList

    def parseHandlers(self,handlerFile,h_type,onlyLoadInit=False,over_write_h_type=None):
        """
        Load method info (name,arg...) in the given handler file
        If onlyLoadInit is True, only the info of __init__ method will be loaded
        If over_write_h_type is given, then over write the handler type with it

        returns a handler object or None if fail to load the given handler file
        """

        # First, let's create a handler object 
        handlerObj = HandlerObject()

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
        # Try to load the handler file
        logging.debug(" -> Loading handler:\t%s" % handlerFile.split('.')[-1])
        try:
            __import__(handlerFile)
        except Exception as e:
            logging.error(" -> Failed to import handler %s : %s" % (handlerFile.split('.')[-1],e))
            logging.debug(traceback.format_exc())
            return handlerObj

        # Find the class object specifies the handler
        handlerModule = sys.modules[handlerFile]
        allClass = inspect.getmembers(handlerModule,inspect.isclass)
        for classObj in allClass:
            # ignore all class objects that are imported or with name starts with underscore
            # if there are multiple classes without underscore, then prefer the one ends with "handler"
            # each handler file should only have one main class not starts with underscore
            # the underscore classes are those used by the main class by not concerned by the user
            if classObj[1].__module__ == handlerFile and not (classObj[0].startswith('_') or \
                re.match('\w+handler', classObj[0])):
                handlerClass = classObj[1]
                break

        # update the handler name and type info
        # handler name is the name of the file
        # handler type is the name of the folder where the file is located for robot independent handlers
        # handler type is robotType.sensor/actuator/init/loco for robot dependent handlers
        handlerObj.name = handlerFile.split('.')[-1]
        if over_write_h_type: 
            handlerObj.type = over_write_h_type
        else:
            handlerObj.type = h_type

        # get all methods in this handler
        handlerMethod = inspect.getmembers(handlerClass,inspect.ismethod)
        # parse each method into method object
        for methodName,method in handlerMethod:
            # only parse the method not start with underscore (exclude __inti__)
            # only parse the __init__ method if required
            if ((not onlyLoadInit and (not str(methodName).startswith('_')) or str(methodName)=='__init__') ):
                # Create a method object and update it
                methodObj = MethodObject()
                methodObj.name = methodName
                methodObj.handler = handlerObj
                for para_name in inspect.getargspec(method)[0]:
                    # create a parameter object if the parameter is not ignorable
                    if para_name not in self.ignore_parameter:
                        paraObj = ParameterObject(para_name)
                        methodObj.para.append(paraObj)
                    else:
                        methodObj.omitPara.append(para_name)

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
                            for paraObj in methodObj.para:
                                # match the definition of a parameter and a parameter object with their names
                                if paraObj.name.lower() == m.group('argName').strip().lower():
                                    # update the parameter object
                                    paraObj.type = m.group('type')
                                    paraObj.des = m.group('description')
                                    if m.group('range') is not None:
                                        try:
                                            for k, v in settingRE.findall(m.group('range')):
                                                if k.lower() not in ['default', 'min', 'max', 'options']:
                                                    logging.warning(' -> Unrecognized setting name "{}" for parameter "{}" of method "{}"'.format(k.lower(), m.group('argName'), methodName))
                                                    continue

                                                setattr(paraObj, k.lower(), json.loads(v.lower()))
                                        except ValueError:
                                            # LOG AN ERROR HERE
                                            logging.error(' -> Could not parse settings for parameter "{}" of method "{}"'.format(m.group('argName'), methodName))

                                        paraObj.resetValue()

                        # If the line comments the function
                        else:
                            methodObj.comment += line + "\n"
                methodObj.comment = methodObj.comment.strip()

                # remove the parameter that has no definition
                argToRemove = []
                for paraObj in methodObj.para:
                    if paraObj.des == None:
                        argToRemove.append(paraObj)
                map(methodObj.para.remove,argToRemove)
                # add this method into the method list of the handler
                handlerObj.methods.append(methodObj)

        return handlerObj

    def printHandler(self):


        for h_type, h_list in self.handler_dic.iteritems():

            print
            print '========Handler for %s  ========' % h_type
            print
            if h_type in self.handler_types:
                for h_obj in h_list:
                    for methodObj in h_obj.methods:
                        print 'Method %s in %s' % (methodObj.name,h_obj.name)
                        print 'Method Description: %s' % methodObj.comment
                        print
                        if len(methodObj.para)==0:
                            print 'The method has no argument.'
                        else:
                            print 'The method has following argument...'
                        print
                        for var in methodObj.para:
                            print '\tArgument Name: %s'%var.name
                            if var.type is not None:
                                print '\tArgument Type: %s'%var.type
                            if var.des is not None:
                                print '\tArgument Description: %s'%var.des
                            if var.default is not None:
                                print '\tThe argument has minimum value of %s, maximum value of %s and default value of %s.'%(var.min,var.max,var.default)
                            print
                        print '================================================='
                        print
            else:
                for robot, h_list in h_list.iteritems():
                    print "For Robot ",robot
                    for h_obj in h_list:
                        for methodObj in h_obj.methods:
                            print 'Method %s in %s' % (methodObj.name,h_obj.name)
                            print 'Method Description: %s' % methodObj.comment
                            print
                            if len(methodObj.para)==0:
                                print 'The method has no argument.'
                            else:
                                print 'The method has following argument...'
                            print
                            for var in methodObj.para:
                                print '\tArgument Name: %s'%var.name
                                if var.type is not None:
                                    print '\tArgument Type: %s'%var.type
                                if var.des is not None:
                                    print '\tArgument Description: %s'%var.des
                                if var.default is not None:
                                    print '\tThe argument has minimum value of %s, maximum value of %s and default value of %s.'%(var.min,var.max,var.default)
                                print
                            print '================================================='
                            print
class RobotFileParser:
    """
    A parser load robot file in each robotFolder
    """
    def __init__(self,path,handler_dic=None):
        self.robots = [] # list of robot objects parsed from robot files
        self.handler_types = ['pose','drive','motionControl','share']       # list of types of handlers, must match with the folder name in lib/handler folder
        self.handler_robotSpecific_type = ['init','locomotionCommand','sensor','actuator']   # list of types of robot specific handlers in each robot folder
        self.ignore_parameter = ['self','initial','proj','shared_data']     # list of name of parameter that should be ignored where parse the handler methods

        self.handler_dic = handler_dic  # dictionary stores all handler information for faster reference
        self.handler_path = path        # handler folder path

    def loadAllRobots(self):
        """
        Load all robot files in the handlers/robots folder
        """
        robotFolderList = os.listdir(os.path.join(self.handler_path,'robots'))
        for robotFolder in robotFolderList:
            # ignore none folder items
            if '.' not in robotFolder:
                fileList = os.listdir(os.path.join(self.handler_path,'robots',robotFolder))
                for fileName in fileList:
                    if fileName.endswith('.robot'):
                        robotObj = self.loadRobotFile(os.path.join(self.handler_path,'robots',robotFolder,fileName))
                        if (robotObj is not None) and (robotObj.type not in [r.type for r in self.robots]):
                            self.robots.append(robotObj)

    def loadRobotFile(self,fileName):
        """
        Given a robot file, return a dictionary holding its information
        """
        logging.debug(" -> Loading robot:\t\t%s" % os.path.basename(fileName).split('.')[0])
        try:
            # try to load the robot file
            robot_data = fileMethods.readFromFile(fileName)
        except IOError:
            logging.ERROR(" -> Cannot load robot: %s" % os.path.basename(fileName).split('.')[0])
            return

        return self.loadRobotData(robot_data)

    def loadRobotData(self,robot_data):
        """
        Given a dictionary of robot handler information, returns a robot object holding all the information
        The dictionary is in the format returned by the readFromFile function
        If the necessary handler of the robot is not specified or can't be loaded, return None
        """
        # create a robot object and update its name and type
        robotObj = RobotObject(r_name=robot_data['RobotName'][0],r_type=robot_data['Type'][0])

        try:
            mat_str = ''.join(robot_data['CalibrationMatrix'])
            if mat_str.strip() == "": raise KeyError()
        except KeyError:
            logging.warning("No calibration data found for robot '%s(%s)'" % (robotObj.name,robotObj.type))
            robotObj.calibrationMatrix = None
        else:
            try:
                # Convert the string form array to array. Trying not to use eval for security problem
                mat_str = mat_str.replace("array(","")
                mat_str = mat_str.replace(")","")
                robotObj.calibrationMatrix = array(ast.literal_eval(mat_str))
            except SyntaxError:
                logging.error("Invalid calibration data found for robot '%s(%s)'" % (robotObj.name,robotObj.type))
                robotObj.calibrationMatrix = None

        robotFolder = robotObj.type

        # load handler configs
        for key,val in robot_data.iteritems():
            if key.endswith('Handler'):
                # find which type of the handler
                handler_type = None
                for h_type in self.handler_types:
                    if key.lower().startswith(h_type.lower()):
                        handler_type = h_type
                        break
                if handler_type == None:
                    # the handler is robot specific
                    for h_type in self.handler_robotSpecific_type:
                        if (key.lower()).startswith(h_type.lower()):
                            handler_type = h_type
                            break
                    if handler_type == None:
                        logging.error("Wrong handler type %s in robot file %s" %(key,robotObj.type+'.robot'))
                        return
                    else:
                        if self.handler_dic is not None:
                            # we can just quick load from the handler dictionary
                            handlerList = self.handler_dic[handler_type][robotFolder]
                        else:
                            handlerParser = HandlerParser(self.handler_path)
                            fileName = '.'.join(['handlers','robots',robotFolder,val[0].split('(')[0]])
                            if handler_type in ['init','locomotionCommand']:
                                onlyLoadInit = True
                            else:
                                onlyLoadInit = False
                            handlerList = [handlerParser.parseHandlers(fileName,handler_type,onlyLoadInit)]

                        # return no robot object if any of the robot specific handler cannot be loaded
                        # since we know there is only one handler object in the list, we can just use index 0 to call it
                        if handlerList[0] == None:
                            logging.error("Cannot load all necessary handlers for robot %s" %robotObj.name)
                            return None
                else:
                    # The handler is robot independent
                    if self.handler_dic is not None:
                        # we can just quick load from the handler dictionary
                        handlerList = self.handler_dic[handler_type]
                    else:
                        handlerParser = HandlerParser(self.handler_path)
                        fileName = '.'.join(['handlers',handler_type,val[0].split('(')[0]])
                        handlerList = [handlerParser.parseHandlers(fileName,handler_type,True)]

                # copy the handler object from the dictionary
                for handlerObj in handlerList:
                    # match the handler object with the name in robot data
                    if handlerObj is not None and handlerObj.name == val[0].split('(')[0]:
                        # Each handler type only has one handler object
                        robotObj.handlers[handler_type] = deepcopy(handlerObj)

                        # overwrite the parameter values
                        para_info = val[0].split('(')[1].replace(')','')

                        for methodObj in robotObj.handlers[handler_type].methods:
                            if methodObj.name == '__init__':
                                initMethodObj = methodObj
                                break
                        for para_name, para_value in re.findall(r'(?P<key>\w+)\s*=\s*(?P<val>"[^"]*"|\'[^\']*\'|[^,]+)', para_info):
                            for paraObj in initMethodObj.para:
                                if para_name == paraObj.name:
                                    paraObj.setValue(para_value)
                                    break
                        break

        return robotObj



class ConfigObject:
    """
    A config file object!
    """
    def __init__(self):
        self.name = None    # name of the config file
        self.robots = []    # list of robot object used in this config file
        self.prop_mapping = {}  # dictionary for storing the propositions mapping
        self.initial_truths = [] # list of initially true propoisitions
        self.region_tags = {} # dictionary mapping tag names to region groups, for quantification
        self.main_robot = '' # name of robot for moving in this config

    def getRobotByName(self, name):
        for r in self.robots:
            if r.name == name:
                return r
        logging.error("Could not find robot of name '%s' in config '%s'." % (name, self.name))
        return None


class ConfigFileParser:
    """
    A parser loads all configuration files
    """

    def __init__(self,config_path,handler_path,proj,handler_dic=None):
        self.proj = proj
        self.config_path = config_path  # config folder path
        self.handler_path = handler_path    # handler folder path
        self.handler_dic = handler_dic  # handler dictionary stores all handler information
        self.configs = []   # list of config object

    def loadAllConfigFiles(self):
        # Create configs/ directory for project if it doesn't exist already
#        config_dir = os.path.join(self.proj.project_root, "configs")
        if not os.path.exists(self.config_path):
            os.mkdir(self.config_path)

        fileList = os.listdir(self.config_path)
        for fileName in fileList:
            if fileName.endswith('.config'):
                configObj = self.loadConfigFile(os.path.join(self.config_path,fileName))
                if configObj is not None:
                    self.configs.append(configObj)

    def loadConfigFile(self,fileName):
        # If only filename offered, assume it is in the config path
        if len(os.path.split(fileName)[0]) == 0:
            fileName = os.path.join(self.config_path,fileName)

        # Add extension to the name if there isn't one.
        if not fileName.endswith('.config'):
            fileName = fileName+'.config'

        logging.debug(" -> Loading config:\t%s" % os.path.basename(fileName).split('.')[0])
        try:
            # First try path relative to project path
            config_data = fileMethods.readFromFile(fileName)
        except IOError:
            logging.ERROR(" -> Cannot load config: %s" % os.path.basename(fileName).split('.')[0])
            return

        # create a config object and update its name
        configObj = ConfigObject()
        try:
            configObj.name = config_data['General Config']['Name'][0]
        except IOError:
            logging.ERROR("Missing general config information in config %s" % os.path.basename(fileName).split('.')[0])

        # parse the string for sensor prop mapping
        for sensorMapping in config_data['General Config']['Sensor_Proposition_Mapping']:
            try:
                sensorProp,sensorFun = [s.strip() for s in sensorMapping.split('=',1)]
            except IOError:
                logging.ERROR("Wrong sensor mapping -- %s" % sensorMapping)

            configObj.prop_mapping[sensorProp]=sensorFun

        # parse the string for actuator prop mapping
        for actuatorMapping in config_data['General Config']['Actuator_Proposition_Mapping']:
            try:
                actuatorProp,actuatorFun = [s.strip() for s in actuatorMapping.split('=',1)]
            except IOError:
                logging.ERROR("Wrong actuator mapping -- %s" % actuatorMapping)
            configObj.prop_mapping[actuatorProp]=actuatorFun

        if 'Initial_Truths' in config_data['General Config']:
            # parse the initially true propositions
            for propName in config_data['General Config']['Initial_Truths']:
                try:
                    configObj.initial_truths.append(propName)
                except IOError:
                    logging.ERROR("Cannot recognize initially true propositions -- %s" %propName)

        if 'Region_Tags' in config_data['General Config']:
            # parse the region tags
            try:
                configObj.region_tags = json.loads("".join(config_data['General Config']['Region_Tags']))
            except ValueError:
                logging.ERROR("Wrong region tags")

        try:
            configObj.main_robot = config_data['General Config']['Main_Robot'][0]
        except (IndexError, KeyError):
            logging.ERROR("Cannot find main robot in config file %s" % fileName)

        # load robot configs
        robot_data = []
        for configKey,configValue in config_data.iteritems():
            if configKey.startswith('Robot'):
                robot_data.append(configValue)

        if robot_data == []:
            logging.ERROR("Missing Robot data in config file %s" % fileName)

        else:
            # using the parsing function in RobotFileParser to parse the data
            robot_parser = RobotFileParser(self.handler_path)
            robot_parser.handler_dic = self.handler_dic
            for data in robot_data:
                try:
                    robotObj = robot_parser.loadRobotData(data)
                    if robotObj is not None:
                        configObj.robots.append(robotObj)
                except IOError:
                    logging.ERROR("Cannot parse robot data in config file %s" % fileName)

        # Missing main robot doesn't affect importing. TODO:Will this create problem?
        """
        # if the main robot for this config cannot be loaded, return no config object
        noRobot = True
        if configObj.main_robot == '' and len(configObj.robots)>0:
            noRobot = False
        if configObj.main_robot != '':
            for robotObj in configObj.robots:
                if configObj.main_robot == robotObj.name:
                    noRobot = False
                    break
        if noRobot:
            if not self.silent: print "WARNING: Cannot load configuration %s, missing main robot object" %fileName
            return None
        """

        return configObj

    def saveAllConfigFiles(self):
        # save all config objects
        savedFileName = []
        for configObj in self.configs:
            self.saveConfigFile(configObj)
            savedFileName.append(configObj.name.replace(' ','_'))
        # remove delected files
        for configFile in os.listdir(self.config_path):
            if configFile.split('.')[0] not in savedFileName:
                os.remove(os.path.join(self.config_path,configFile))


    def saveConfigFile(self,configObj,fileName=''):
        """
        Write all data out to a file.
        """

        if fileName is '':
            fileName = configObj.name.replace(' ','_')


         # Add extension to the name if there isn't one.
        if not fileName.endswith('.config'):
            fileName = fileName+'.config'

        data = {'General Config':{'Name':configObj.name}}

        # proposition mapping
        sensorMappingList = []
        actuatorMappingList = []
        for prop, fun in configObj.prop_mapping.iteritems():
            if 'sensor' in fun.lower():
                sensorMapping = prop + ' = ' + fun
                sensorMappingList.append(sensorMapping)
            elif 'actuator' in fun.lower():
                actuatorMapping = prop + ' = ' + fun
                actuatorMappingList.append(actuatorMapping)

        data['General Config']['Sensor_Proposition_Mapping'] = sensorMappingList
        data['General Config']['Actuator_Proposition_Mapping'] = actuatorMappingList
        data['General Config']['Main_Robot'] = configObj.main_robot
        data['General Config']['Initial_Truths'] = configObj.initial_truths
        data['General Config']['Region_Tags'] = json.dumps(configObj.region_tags)

        for i,robot in enumerate(configObj.robots):
            header = 'Robot'+str(i+1)+' Config'
            data[header]={}
            data[header]['RobotName'] = robot.name
            data[header]['Type'] = robot.type

            if robot.calibrationMatrix is not None:
                data[header]['CalibrationMatrix'] = repr(robot.calibrationMatrix)

            data[header]['InitHandler'] = robot.handlers['init'].toString()
            data[header]['PoseHandler'] = robot.handlers['pose'].toString()
            data[header]['MotionControlHandler'] = robot.handlers['motionControl'].toString()
            data[header]['DriveHandler'] = robot.handlers['drive'].toString()
            data[header]['LocomotionCommandHandler'] = robot.handlers['locomotionCommand'].toString()
            data[header]['SensorHandler'] = robot.handlers['sensor'].toString()
            data[header]['ActuatorHandler'] = robot.handlers['actuator'].toString()


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
                    "CalibrationMatrix": "3x3 matrix for converting coordinates, stored as lab->map",
                    "Actuator_Proposition_Mapping": 'Mapping between actuator propositions and actuator handler functions',
                    "Sensor_Proposition_Mapping": "Mapping between sensor propositions and sensor handler functions",
                    "Name": 'Configuration name',
                    "Main_Robot":'The name of the robot used for moving in this config',
                    "Initial_Truths": "Initially true propositions",
                    "Region_Tags": "Mapping from tag names to region groups, for quantification"}

        fileMethods.writeToFile(os.path.join(self.config_path,fileName), data, comments)



if __name__ == '__main__':
    proj = project.Project()
    proj.ltlmop_root = '/home/jim/Desktop/ltlmop_git/src'
    proj.project_root = '/home/jim/Desktop/ltlmop_git/src/examples/newSensorTest'
    h = HandlerSubsystem(proj)
    h.loadAllHandlers()
    h.loadAllRobots()
    h.loadAllConfigFiles()


    #h.handler_parser.printHandler()
    #print h.configs[0].robots[0].handlers['sensor'].methods

    testStringBefore = 'share.dummySensor.buttonPress(button_name="Wave")'
    testMethod = h.string2Method(testStringBefore)

    print
    testStringAfter = h.method2String(testMethod,'share')

    print testStringBefore == testStringAfter

    testStringBefore = 'MAE.naoSensor.hearWord(word="Fire",threshold=0.9)'
    testMethod = h.string2Method(testStringBefore)
    testStringAfter = h.method2String(testMethod,'MAE')
    print testStringBefore == testStringAfter

