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
        self.configs = []           # list of config objects

        # Create Handler path
        self.handler_path = os.path.join('lib','handlers')
        # Create config path
        self.config_path = os.path.join(self.proj.project_root,'configs')

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
            handler_config.robot_type = r_type
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
                logging.warning("Cannot find handler config with handler type {!r} for robot {!r}.\n \
                                It is possible the handler config was not successfully loaded." \
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

        # this list stores the experiment config names that are not loaded successfully
        self.configs_incomplete = []

        for file_name in os.listdir(self.config_path):
            if file_name.endswith('.config'):
                experiment_config = ExperimentConfig()
                try:
                    experiment_config.fromFile(os.path.join(self.config_path,file_name), self)
                except ht.LoadingError, msg:
                    logging.warning(str(msg) + ' in experiment config file {!r}.'\
                                    .format(os.path.join(self.config_path, file_name)))
                    self.configs_incomplete.append(os.path.join(self.config_path,file_name))
                    continue
                except TypeError as e:
                    logging.error(e)
                    self.configs_incomplete.append(os.path.join(self.config_path,file_name))
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

    def string2Method(self, method_string, robots):
        """
        Return the HandlerMethodConfig according to the input string from configEditor
        This functions must be located in HandlerSubsystem in order to get access to handler_configs dictionary

        method_string: a string that stores the information of a method configObj
                       for example: a = "nao_maeby.NaoSensorHandler.seePerson(name='nao', test=[1,2,3])"
        robots: a list of robots in the current experiment config. we need this to find the robot type and avaliable handlers
                based on robot_name
        """

        if not self.handler_configs:
            logging.error("Cannot find handler_configs dictionary, please load all handlers first.")
            return None

        # construct regex for identify each parts
        method_RE = re.compile("(?P<robot_name>\w+)\.(?P<handler_name>\w+)\.(?P<method_name>\w+)\((?P<para_info>.*?)\)")

        result = method_RE.search(method_string)

        if not result:
            logging.error("Cannot parse setting {!r}".format(method_string))
            return None

        # parse each part
        robot_name = result.group("robot_name")
        handler_name = result.group("handler_name")
        method_name = result.group("method_name")
        para_info = result.group("para_info")

        # find which handler does this method belongs to
        handler_config = None

        if robot_name == "share":
            # this ia a dummy sensor/actuator
            for h in self.handler_configs["share"][ht.SensorHandler] + \
                     self.handler_configs["share"][ht.ActuatorHandler]:
                if h.name == handler_name:
                    handler_config = h
                    break
        else:
            # this is a robot sensor/actuator
            for robot_config in robots:
                if robot_config.name == robot_name:
                    handler_config = robot_config.getHandlerByName(handler_name)
                    break

        # we did not find the correct handler config
        if handler_config is None:
            logging.error("Cannot recognize handler {!r} of robot {!r}".format(handler_name, robot_name))
            logging.error("Please make sure it is correctly loaded")
            return None

        # try to find the method config
        try:
            method_config = deepcopy(handler_config.getMethodByName(method_name))
        except ValueError:
            return None

        # update parameters of this method
        method_config.updateParaFromString(para_info)

        return method_config


    def method2String(self, method_config, robot_name=''):
        """
        Return the string representation according to the input method config
        """
        if not self.handler_configs:
            logging.error("Cannot find handler dictionary, please load all handler first.")
            return
        if not isinstance(method_config,HandlerMethodConfig):
            logging.error("Input is not a valid method config.")
            return
        if robot_name=='':
            logging.error("Needs robot name for method2String")
            return

        handler_name = method_config.handler.name
        method_name = method_config.name

        # convert all parameter object into string
        para_list = []
        for para_config in method_config.para:
            if para_config.value is None:
                para_list.append( para_config.name+'='+str(para_config.default))
            else:
                if para_config.para_type.lower() in ['str', 'string', 'region']:
                    para_list.append( para_config.name+'=\"'+str(para_config.value)+'\"')
                else:
                    para_list.append( para_config.name+'='+str(para_config.value))

        para_info = ','.join(para_list)

        return '.'.join([robot_name,handler_name,method_name])+'('+para_info+')'


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

    def saveAllConfigFiles(self):
        # save all config object
        saved_file_name = []
        for experiment_config in self.configs:
            logging.debug("Saving config file {0}".format(experiment_config.file_name))
            if experiment_config.saveConfig():
                # successfully saved
                logging.debug("Config file {0} successfully saved.".format(experiment_config.file_name))
                saved_file_name.append(experiment_config.file_name)
            else:
                logging.error("Could not save config file {0}".format(experiment_config.file_name))

        # remove deleted files
        # do not delete unsuccessfully loaded configs
        for config_file in os.listdir(self.config_path):
            if (os.path.join(self.config_path, config_file) not in saved_file_name) \
                    and (os.path.join(self.config_path, config_file) not in self.configs_incomplete):
                os.remove(os.path.join(self.config_path, config_file))


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
