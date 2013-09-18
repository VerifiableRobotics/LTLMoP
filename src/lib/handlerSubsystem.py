#!/usr/bin/env python


""" ================================================
    handlerSubsystem.py - Interface for working with handlers, configs
    ================================================
"""


import os, sys, re
import fileMethods
import inspect,types
from numpy import *
from copy import deepcopy
import project
import ast
import json
import traceback
import globalConfig, logging
from lib.hsubConfigObjects import MethodParameterConfig,HandlerMethodConfig,\
                                HandlerConfig,RobotConfig,ExperimentConfig
import lib.handlers.handlerTemplates as ht


###################################################
# Define individual objects for handler subsystem #
###################################################

class HandlerSubsystem:
    """
    Interface dealing with configuration files and handlers
    """
    def __init__(self,proj):
        self.proj = proj

        self.handler_configs = {}   # dictionary for all handler information [robot type or shared][handler type]
        self.robot_configs = []     # list of robot objects
        self.robots = []            # list of robot objects
        self.configs = []           # list of config objects

        # Create Handler path
        self.handler_path = os.path.join('lib','handlers')

        # Create robot parser
        self.robot_parser = RobotFileParser(self.handler_path)

        # Create config parser
        self.config_path = os.path.join(self.proj.project_root,'configs')
        self.config_parser = ConfigFileParser(self.config_path,self.handler_path,self.proj)

    def _getSubdirectories(self, path):
        """
        Return subdirectories in `path` relative to ltlmop root.
        Only goes one level.
        """
        abs_path = os.path.join(globalConfig.get_ltlmop_root(), path)
        return [os.path.join(path, item) for item in os.listdir(abs_path) \
                if os.path.isdir(os.path.join(abs_path, item))]

    def loadAllHandlers(self):
        """
        Load all handlers in the handler folder
        """
        # get a list of folders in lib/handlers/ directory
        handler_folders = self._getSubdirectories(self.handler_path)
        try:
            handler_folders.remove(os.path.join(self.handler_path, 'share'))
        except ValueError:
            logging.warning('No shared handler directory found in {!r}'.format(self.handler_path))
        else:
            handler_folders.extend(self._getSubdirectories(os.path.join(self.handler_path, 'share')))
            
        for folder in handler_folders:
            for handler_file in os.listdir(os.path.join(globalConfig.get_ltlmop_root(), folder)):
                abs_path = os.path.join(globalConfig.get_ltlmop_root(), folder, handler_file)

                # find all handler files and ignore internal files
                if not (os.path.isfile(abs_path) and handler_file.endswith('.py') and not handler_file.startswith('_')): 
                    continue

                module_info = re.split(r"[\\/]", folder)
                robot_type = module_info[2]

                # handler type
                if len(module_info) == 4:
                    # this is a shared handler
                    h_type = module_info[3]
                else:
                    # this a robot handler
                    h_type = None

                # handler name
                h_name = os.path.splitext(handler_file)[0]

                handler_config = self.loadHandler(robot_type, h_type, h_name)
                if handler_config is None:
                    # the handler cannot be loaded
                    continue

                # save it into the dictionary
                if robot_type not in self.handler_configs.keys():
                    self.handler_configs[robot_type] = {}
                if handler_config.h_type not in self.handler_configs[robot_type].keys():
                    self.handler_configs[robot_type][handler_config.h_type] = []
                self.handler_configs[robot_type][handler_config.h_type].append(handler_config)

    def loadHandler(self, r_type, h_type, h_name):
        """
        Load the handler config object from the file based on the given info
        """
        # create a handler config object first
        handler_config = HandlerConfig()

        if r_type in ['share']:
            # this is a shared handler, we will require a handler type
            # if the h_type is a handler type class object, translate it into a str
            if not isinstance(h_type, str):
                h_type = ht.getHandlerTypeName(h_type)
            handler_module = '.'.join(['lib', 'handlers', r_type, h_type, h_name])
        else:
            # this is a robot handler, no handler type required
            handler_module = '.'.join(['lib', 'handlers', r_type, h_name])

        try:
            handler_config.parseHandler(handler_module)
        except ImportError as import_error:
            # TODO: Log an error here if the handler is necessary
            handler_config = None

        return handler_config

    def getHandlerConfigDefault(self, r_type, h_type, h_name):
        """
        Get default handler config object from handler_configs if exists
        Or load it from corresponding file if not exits
        given rtype (str: share or robot type (str)), htype (class), h_name (str). NOT yet overridden by .robot defaults
        """

        default_handler_config = None

        if self.handler_configs == {}:
            # if no handler has been loaded yet we will load the handler from file
            default_handler_config = self.loadHandler(r_type, h_type, h_name)
        else:
            # fetch it from the exiting handler_configs dictionary
            if r_type not in self.handler_configs.keys():
                # robot type is not recognized
                logging.warning("Cannot find handler config with robot type {!r}.".format(r_type))
            elif h_type not in self.handler_configs[r_type].keys():
                # handler type is not recognized
                logging.warning("Cannot find handler config with handler type {!r} for robot {!r}." \
                                .format(ht.getHandlerTypeName(h_type), r_type))
            else:
                for handler_config in self.handler_configs[r_type][h_type]:
                    if handler_config.name == h_name:
                        # we found the handler config object
                        default_handler_config = deepcopy(handler_config)
                        break

                if default_handler_config is None:
                    # Cannot find handler config object with given name
                    logging.warning("Cannot find handler config with handler name {!r}.".format(h_name))

        return default_handler_config

    def loadAllRobots(self):
        """
        Load all robot files in each handlers/robot_type folder
        """
        # get a list of folders in lib/handlers/ directory
        robot_folders = self._getSubdirectories(self.handler_path)
        try:
            robot_folders.remove(os.path.join(self.handler_path, 'share'))
        except ValueError: pass

        for robot_folder in robot_folders:
            # find all robot config files
            robot_files = [f for f in os.listdir(robot_folder) if f.endswith('.robot')]
            for robot_file in robot_files:
                robot_config = RobotConfig()
                # now load the file
                try:
                    robot_config.fromFile(os.path.join(robot_folder, robot_file), self)
                except ht.LoadingError, msg:
                    logging.warning(str(msg) + ' in robot file {!r}.'.format(os.path.join(robot_folder, robot_file)))
                    continue
                except TypeError:
                    continue
                else:
                    self.robot_configs.append(robot_config)

    def loadAllConfigFiles(self):
        """
        Load all experiment config files in the project/configs folder
        """
        # Create configs/ directory for project if it doesn't exist already
        if not os.path.exists(self.config_path):
            os.mkdir(self.config_path)

        for file_name in os.listdir(self.config_path):
            if file_name.endswith('.config'):
                experiment_config = ExperimentConfig()
                try:
                    experiment_config.fromFile(os.path.join(self.config_path,file_name), self)
                except ht.LoadingError, msg:
                    logging.warning(str(msg) + ' in experiment config file {!r}.'\
                                    .format(os.path.join(self.config_path, file_name)))
                    continue
                except TypeError as e:
                    logging.error(e)
                else:
                    self.configs.append(experiment_config)

    def getRobotByType(self, t):
        """
        Assume only one robot is loaded per type
        """
        for r in self.robot_configs:
            if r.r_type == t:
                return r
        logging.error("Could not find robot of type '{0}'".format(t))
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

        if not self.handler_dic:
            logging.error("Cannot find handler dictionary, please load all handlers first.")
            return

        method_info,para_info = method_string.split('(')
        para_info = para_info.replace(')','')

        items = method_info.split('.')
        methodObj = HandlerMethodConfig()

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
                        
                        break

            if handlerObj is None:
                logging.error("Cannot recognize robot {}".format(robotName)) 
                logging.error("I only know about these robots: {!r}".format([r.name for r in self.robots]))
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
        if not self.handler_dic:
            logging.error("ERROR: Cannot find handler dictionary, please load all handler first.")
            return
        if not isinstance(methodObj,HandlerMethodConfig):
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
                logging.info("Loading handler: %s" % fileName.split('.')[-1])
                try:
                    __import__(fileName)
                except ImportError as import_error:
                    logging.error("Failed to import handler %s : %s" % (fileName.split('.')[-1],import_error))

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
        logging.debug("Loading robot: %s" % os.path.basename(fileName).split('.')[0])
        try:
            # try to load the robot file
            robot_data = fileMethods.readFromFile(fileName)
        except IOError:
            logging.ERROR("Cannot load robot: %s" % os.path.basename(fileName).split('.')[0])
            return

        robotObj = self.loadRobotData(robot_data)

        for handler_type in robotObj.handlers.keys():
            if robotObj.handlers[handler_type] is None:
                logging.warning("Default {} handler specified in .robot file for {!r} could not be loaded".format(handler_type, robotObj.type))

        return robotObj


    def loadRobotData(self,robot_data):
        """
        Given a dictionary of robot handler information, returns a robot object holding all the information
        The dictionary is in the format returned by the readFromFile function
        If the necessary handler of the robot is not specified or can't be loaded, return None
        """
        # create a robot object and update its name and type
        robotObj = RobotConfig(r_name=robot_data['RobotName'][0],r_type=robot_data['Type'][0])

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
                        if self.handler_dic:
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
                    if self.handler_dic:
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



class ConfigFileParser:
    """
    A parser loads all configuration files
    """

    def __init__(self,config_path,handler_path,proj,handler_dic=None):
        self.proj = proj
        self.config_path = config_path  # config folder path
        self.handler_path = handler_path    # handler folder path
        self.handler_dic = handler_dic  # handler dictionary stores all handler information
        self.configs = []   # list of complete config object
        self.configs_incomplete = []    # list of incompletely loaded configs

    def loadAllConfigFiles(self):
        # Create configs/ directory for project if it doesn't exist already
        if not os.path.exists(self.config_path):
            os.mkdir(self.config_path)

        fileList = os.listdir(self.config_path)
        for fileName in fileList:
            if fileName.endswith('.config'):
                configObj = self.loadConfigFile(os.path.join(self.config_path,fileName))
                # save the config object to the corrsponding list
                if configObj.complete:
                    self.configs.append(configObj)
                else:
                    self.configs_incomplete.append(configObj)

    def loadConfigFile(self,fileName):
        # If only filename offered, assume it is in the config path
        if len(os.path.split(fileName)[0]) == 0:
            fileName = os.path.join(self.config_path,fileName)

        # Add extension to the name if there isn't one.
        if not fileName.endswith('.config'):
            fileName = fileName+'.config'
        
        logging.debug("Loading config: {}".format(os.path.basename(fileName).split('.')[0]))
        # create a config object and update its name and fileName
        configObj = ExperimentConfig()
        configObj.fileName= fileName
        try:
            # First try path relative to project path
            config_data = fileMethods.readFromFile(fileName)
        except IOError:
            logging.ERROR("Cannot load config: {}".format(fileName))
            return configObj

        configObj.complete = True

        try:
            configObj.name = config_data['General Config']['Name'][0]
        except IOError:
            logging.warning("Missing general config information in config {}".format(os.path.basename(fileName).split('.')[0]))
            configObj.complete = False

        # parse the string for sensor prop mapping
        if 'Sensor_Proposition_Mapping' in config_data['General Config']:
            for sensorMapping in config_data['General Config']['Sensor_Proposition_Mapping']:
                try:
                    sensorProp,sensorFun = [s.strip() for s in sensorMapping.split('=',1)]
                except IOError:
                    logging.warning("Wrong sensor mapping -- {}".format(sensorMapping))
                    configObj.complete = False
                configObj.prop_mapping[sensorProp]=sensorFun
        else:
            logging.warning("Cannot find sensor proposition mapping in config file: {}".format(fileName))
            configObj.complete = False


        # parse the string for actuator prop mapping
        if 'Actuator_Proposition_Mapping' in config_data['General Config']:
            for actuatorMapping in config_data['General Config']['Actuator_Proposition_Mapping']:
                try:
                    actuatorProp,actuatorFun = [s.strip() for s in actuatorMapping.split('=',1)]
                except IOError:
                    logging.warning("Wrong actuator mapping -- {}".format(actuatorMapping))
                    configObj.complete = False
                configObj.prop_mapping[actuatorProp]=actuatorFun
        else:
            logging.warning("Cannot find actuator proposition mapping in config file: {}".format(fileName))
            configObj.complete = False


        if 'Initial_Truths' in config_data['General Config']:
            # parse the initially true propositions
            for propName in config_data['General Config']['Initial_Truths']:
                try:
                    configObj.initial_truths.append(propName)
                except IOError:
                    logging.warning("Cannot recognize initially true propositions -- {}".format(propName))
                    configObj.complete = False
        else:
            logging.warning("Cannot find initial truth proposition in config file: {}".format(fileName))
            configObj.complete = False

        if 'Region_Tags' in config_data['General Config']:
            # parse the region tags
            try:
                configObj.region_tags = json.loads("".join(config_data['General Config']['Region_Tags']))
            except ValueError:
                logging.warning("Wrong region tags")
                configObj.complete = False

        if 'Main_Robot' in config_data['General Config']:
            # Load main robot name
            try:
                configObj.main_robot = config_data['General Config']['Main_Robot'][0]
            except (IndexError, KeyError):
                logging.warning("Cannot parse main robot name {0} in config file {1}".format(config_data['General Config']['Main_Robot'],fileName))
                configObj.complete = False
        else:
            logging.warning("Cannot find main robot in config file: {}".format(fileName))
            configObj.complete = False

        # load robot configs
        robot_data = []
        for configKey,configValue in config_data.iteritems():
            if configKey.startswith('Robot'):
                robot_data.append(configValue)

        if robot_data == []:
            logging.warning("Missing Robot data in config file {}".format(fileName))
        else:
            # using the parsing function in RobotFileParser to parse the data
            robot_parser = RobotFileParser(self.handler_path)
            robot_parser.handler_dic = self.handler_dic
            for data in robot_data:
                try:
                    robotObj = robot_parser.loadRobotData(data)
                    # TODO chage this to check uncomplete obj
                    if robotObj is not None:
                        configObj.robots.append(robotObj)
                except IOError:
                    logging.warning("Cannot parse robot data in config file %s" % fileName)

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
            logging.debug("Saving config file {0}".format(configObj.fileName))
            if configObj.saveConfig():
                # successfully saved
                logging.debug("Config file {0} successfully saved.".format(configObj.fileName))
            else:
                logging.error("Could not save config file {0}".format(configObj.fileName))
            savedFileName.append(configObj.fileName)
        
        # construct a list of config filenames that are not loaded successfully, so that we don't delete them
        incomplete_list = [c.fileName for c in self.configs_incomplete]

        # remove delected files
        for configFile in os.listdir(self.config_path):
            if (os.path.join(self.config_path,configFile) not in savedFileName) or (os.path.join(self.config_path,configFile) not in savedFileName): 
                os.remove(os.path.join(self.config_path,configFile))


if __name__ == '__main__':
    pass
#    m = HandlerMethodConfig()
#    print m
#    m = MethodParameterConfig('Jim')
#    print m
#    m = HandlerConfig()
#    print m
#    m = RobotConfig()
#    print m
#    m = ExperimentConfig()
#    print m
    proj = project.Project()
    proj.project_root = '/home/jim/LTLMoP/src/examples/firefighting/'
    h = HandlerSubsystem(proj)
#    h.loadAllHandlers()

#    print [hcfg.name for hcfg in h.handler_configs['share'][ht.SensorHandler]]

#    h.loadAllRobots()
 #   print h.robot_configs
    h.loadAllConfigFiles()
    print h.configs


#    h.handler_parser.printHandler()
#    print h.configs[0].robots[0].handlers['sensor'].methods

#    testStringBefore = 'share.dummySensor.buttonPress(button_name="Wave")'
#    testMethod = h.string2Method(testStringBefore)

#    print
#    testStringAfter = h.method2String(testMethod,'share')

#    print testStringBefore == testStringAfter

#    testStringBefore = 'MAE.naoSensor.hearWord(word="Fire",threshold=0.9)'
#    testMethod = h.string2Method(testStringBefore)
#    testStringAfter = h.method2String(testMethod,'MAE')
#    print testStringBefore == testStringAfter
