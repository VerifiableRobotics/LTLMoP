#!/usr/bin/env python


""" ================================================
    handlerSubsystem.py - Interface for working with handlers, configs
    ================================================
"""


import os
import sys

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

import re
import time
import fileMethods
from copy import deepcopy
import project
import globalConfig, logging
from hsubConfigObjects import MethodParameterConfig,HandlerMethodConfig,\
                                HandlerConfig,RobotConfig,ExperimentConfig
import handlers.handlerTemplates as ht
from hsubParsingUtils import parseCallString

# TODO: Get rid of this todo list
# TODO: Move testing code to doctest
# TODO: Implement motion handler wrapper
# TODO: Group functions better
# TODO: Robot type not recognized when not loaded
# TODO: warn if no __init__.py is in a robot folder
# TODO: make sure python defaults matched doc defaults for para
# TODO: pass executor to handlers, and use postEvent
# TODO: clean up toString
# TODO: test multiple shared handlers
# TODO: clean relation of motionControl, drive, LocomotionCommand
# TODO: check proposition name with enabled s/a
# TODO: reorder the load config fromData function if indent
# TODO: print out which file is loading


###################################################
# Define individual objects for handler subsystem #
###################################################

class HandlerSubsystem:
    """
    Interface dealing with configuration files and handlers
    """

    def __init__(self, executor, project_root_dir):
        self.executor = executor

        self.handler_configs = {}   # dictionary for all handler information [robot type or shared][handler type]
        self.robot_configs = []     # list of robot objects
        self.configs = []           # list of config objects
        self.executing_config = None  # current experiment config object that is executing

        self.prop2func = {}         # a mapping from a proporsition to a handler function for execution
        self.handler_instance = []  # a list of handler instances that are instantiated
        self.method_configs = set() # a set of func references

        self.coordmap_map2lab = None# function that maps from map coord to lab coord
        self.coordmap_lab2map = None# function that maps from lab coord to map coord

        # Create Handler path
        self.handler_path = os.path.join('lib','handlers')
        # Create config path
        self.config_path = os.path.join(project_root_dir,'configs')

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

    def findHandlerTypeStringFromName(self, handler_name):
        """
        given a handler name, find folder of this handler in the handlers/share folder
        """
        share_folder = os.path.join(self.handler_path, "share")

        for handler_type_string in os.listdir(share_folder):
            temp_folder = os.path.join(share_folder, handler_type_string)
            if os.path.isdir(temp_folder):
                handler_files = [f for f in os.listdir(temp_folder) if f.endswith('.py')]
                if handler_name + ".py" in handler_files:
                    return handler_type_string
        logging.error("Cannot find the type of handler with name {} in folds in handlers/share".format(handler_name))
        return None

    def loadHandler(self, r_type, h_type, h_name):
        """
        Load the handler config object from the file based on the given info
        """
        # create a handler config object first
        handler_config = HandlerConfig()

        if r_type in ['share']:
            # this is a shared handler, we will require a handler type
            # if the h_type is a handler type class object, translate it into a str
            if h_type is None:
                h_type =  self.findHandlerTypeStringFromName(h_name)
            if not isinstance(h_type, basestring):
                h_type = ht.getHandlerTypeName(h_type)
            handler_module = '.'.join(['lib', 'handlers', r_type, h_type, h_name])
        else:
            # this is a robot handler, no handler type required
            handler_module = '.'.join(['lib', 'handlers', r_type, h_name])

        try:
            if h_type in ("Sensor", "Actuator"):
                handler_config.loadHandlerMethod(handler_module)
            else:
                handler_config.loadHandlerMethod(handler_module, onlyLoadInit=True)

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

    def setExecutingConfig(self, config_object_name):
        """
        set the current executing config to the experiment config with the given name
        if the given config cannot be found, try to load it first
        """
        self.executing_config = None
        for config_object in self.configs:
            if config_object_name == config_object.name:
                self.executing_config = config_object

        if self.executing_config is None:
            config, success = self.loadConfigFile(config_object_name)
            if success:
                self.configs.append(config)
                self.setExecutingConfig(config_object_name)
            else:
                logging.error("Cannot find the config with name {}".format(config_object_name))

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
            config, success = self.loadConfigFile(file_name)
            if success:
                self.configs.append(config)
            else:
                self.configs_incomplete.append(config)

    def loadConfigFile(self, file_name):
        """
        Load the given experiment config file in the project/configs folder
        """
        file_name = file_name.replace(" ","_")
        if not file_name.endswith(".config"):
            file_name = file_name + ".config"

        experiment_config = ExperimentConfig()
        try:
            experiment_config.fromFile(os.path.join(self.config_path,file_name), self)
        except ht.LoadingError, msg:
            logging.warning(str(msg) + ' in experiment config file {!r}.'\
                            .format(os.path.join(self.config_path, file_name)))
            return os.path.join(self.config_path,file_name), False
        except TypeError as e:
            logging.error(e)
            return os.path.join(self.config_path,file_name), False
        else:
            return experiment_config, True

    def getRobotByType(self, t):
        """
        Assume only one robot is loaded per type
        """
        for r in self.robot_configs:
            if r.r_type == t:
                return r
        logging.error("Could not find robot of type '{0}'".format(t))
        return None

    def getHandlerInstanceByName(self, handler_name):
        """
        Return the instantiated handler object for a given name or None if it is not instantiated
        """
        for h in self.handler_instance:
            if h.__class__.__name__ == handler_name:
                return h
        return None

    def getMainRobot(self):
        """
        Return the main robot of the current executing config
        """
        if self.executing_config == None:
            raise ValueError("Cannot find executing config for handlersubsystem")
        # get the main robot config
        return self.executing_config.getRobotByName(self.executing_config.main_robot)

    def getHandlerInstanceByType(self, handler_type_class, robot_name = ""):
        """
        Return the handler instance of the given handler type
        When no robot_name is given, the handler is assumed to be of the main robot
        When the handler type is either sensor or actuator, a robot_name is required
        Return None if the handler instance cannot be found
        """
        if robot_name == "":
            # no robot is specified
            if handler_type_class in [ht.SensorHandler, ht.ActuatorHandler]:
                raise ValueError("A robot name is required when looking for instance of sensor or actuator handler")
            else:
                # we assume it is asking for the handler of main robot
                robot_config = self.getMainRobot()
        else:
            # find the robot config of the given name
            robot_config = self.executing_config.getRobotByName(robot_name)

        # now look for the handler instance of the given type
        handler_instance = self.getHandlerInstanceByName(robot_config.getHandlerOfRobot(handler_type_class).name)

        return handler_instance

    def setVelocity(self, x, y):
        """
        a wrapper function that set the velocity to the drive handler of the main robot
        """
        # get the drive handler
        drive_handler_instance = self.getHandlerInstanceByType(ht.DriveHandler)

        if drive_handler_instance is None:
            raise ValueError("Cannot set Velocity, because no drive handler instance is found for the main robot")

        # set the velocity
        drive_handler_instance.setVelocity(x, y)

    def gotoRegion(self, current_region, next_region):
        """
        a wrapper function that set the target region to the motionControl handler of the main robot
        """
        # get the motionControl handler
        motion_handler_instance = self.getHandlerInstanceByType(ht.MotionControlHandler)

        # if the region is object, we need to find the index of it
        if not isinstance(current_region, int):
            current_region = self.executor.proj.rfi.regions.index(current_region)
        if not isinstance(next_region, int):
            next_region = self.executor.proj.rfi.regions.index(next_region)

        if motion_handler_instance is None:
            raise ValueError("Cannot set target region, because no motionControl handler instance is found for the main robot")

        # set the target region
        arrived = motion_handler_instance.gotoRegion(current_region, next_region)

        return arrived

    def getPose(self, cached=False):
        """
        A wrapper function that returns the pose from the pose handler of the main robot in the
        current executing config
        """
        # get the main robot config
        robot_config = self.getMainRobot()

        # first make sure the coord transformation function is ready
        if self.coordmap_map2lab is None:
            self.coordmap_map2lab, self.coordmap_lab2map = robot_config.getCoordMaps()
            self.executor.proj.coordmap_map2lab, self.executor.proj.coordmap_lab2map = robot_config.getCoordMaps()

        pose_handler_instance = self.getHandlerInstanceByType(ht.PoseHandler)

        if pose_handler_instance is None:
            raise ValueError("Cannot get current pose, because no pose handler instance is found for the main robot")

        return pose_handler_instance.getPose(cached)

    def getDefaultPropMapping(self, sensor_prop_list, actuator_prop_list):
        """
        Return the default proposition mapping based on the propositions given
        """

        # we need handler configs to create the default proposition mapping
        if self.handler_configs == {}:
            raise ValueError("Cannot get default proposition mapping. Please load all handlers first.")

        prop_mapping = {}
        for p in sensor_prop_list:
            m = deepcopy(self.handler_configs["share"][ht.SensorHandler][0].getMethodByName("buttonPress"))
            para = m.getParaByName("button_name")
            para.setValue(p)
            prop_mapping[p] = self.method2String(m, "share")

        for p in actuator_prop_list:
            m = deepcopy(self.handler_configs["share"][ht.ActuatorHandler][0].getMethodByName("setActuator"))
            para = m.getParaByName("name")
            para.setValue(p)
            prop_mapping[p] = self.method2String(m, "share")

        return prop_mapping


    def initializeAllMethods(self):
        """
        initialize all method in self.prop2func mapping with initial=True
        """

        logging.info("Initializing sensor/actuator methods...")
        # initialize all sensor and actuators
        # since we cannot distinguish the method for sensor and actuator
        # we will pass in arguments for both types of methods
        for method_config in self.method_configs:
            if method_config.handler.h_type is ht.SensorHandler:
                method_config.execute(initial=True)
            elif method_config.handler.h_type is ht.ActuatorHandler:
                method_config.execute(initial=True, actuatorVal=False)
            else:
                raise TypeError("Handler mappings must contain only Sensor or Actuator handlers")

    def prepareHandler(self, handler_config):
        """
        Instantiate the handler object of the given handler config if it is not already instantiated
        Return the handler instance
        """

        # Check if we alreay instantiated the handler
        handler_name = handler_config.name
        h = self.getHandlerInstanceByName(handler_name)

        if h is None:
            # we need to instantiate the handler
            robot_type = handler_config.robot_type

            # construct the handler module path for importing
            if robot_type == "share":
                handler_type_string = ht.getHandlerTypeName(handler_config.h_type)
                handler_module_path = ".".join(["lib", "handlers", robot_type, handler_type_string,  handler_name])
            else:
                handler_module_path = ".".join(["lib", "handlers", robot_type, handler_name])

            # find the handler class object
            h_name, h_type, handler_class = HandlerConfig.loadHandlerClass(handler_module_path)

            # get the HMC for the init_method
            init_method_config = handler_config.getMethodByName("__init__")

            # construct the arguments list; everyone gets a reference to executor
            arg_dict = {"executor": self.executor}

            # everyone except for InitHandler gets shared_data too
            if h_type is not ht.InitHandler:
                arg_dict.update({"shared_data":self.executor.proj.shared_data})

            # add any arguments specific to this method
            arg_dict.update(init_method_config.getArgDict())

            # instantiate the handler
            try:
                h = handler_class(**arg_dict)
            except Exception:
                logging.exception("Failed during handler {} instantiation".format(handler_module_path))
            else:
                self.handler_instance.append(h)
        return h

    def prepareMapping(self):
        """
        prepare the python objects corresponding to each proposition stored in the prop_mapping of current config object
        """

        mapping = self.executing_config.prop_mapping

        for prop_name, func_string in mapping.iteritems():
            if prop_name in self.executor.proj.all_sensors:
                mode = "sensor"
            elif prop_name in self.executor.proj.all_actuators:
                mode = "actuator"
            else:
                raise ValueError("Proposition name {} is not recognized.".format(prop_name))

            self.prop2func[prop_name] = self.createPropositionMappingExecutionFunctionFromString(func_string, mode)

    def _makeHandlerMethodConfigAndGetExecutionFunction(self, call_descriptor):
        """ A helper function for createPropositionMappingFunctionFromString
            that will get called as the string is parsed. """

        # Sanity check the name
        if len(call_descriptor.name) < 3:
            raise SyntaxError("Handler method name {!r} too short.", ".".join(call_descriptor.name))

        # Create a new HMC
        hmc = self.createHandlerMethodConfig(*call_descriptor.name, kwargs=call_descriptor.args)

        # Save it to a list so we can initialize it later
        self.method_configs.add(hmc)

        # Return a reference to this HMC's execute function
        return hmc.execute

    def createPropositionMappingExecutionFunctionFromString(self, func_string, mode):
        """ Given a string description of the function(s) that some
            proposition maps to, create the appropriate HandlerMethodConfigs
            and return the function that will evaluate their execute() methods. """

        # Call the parser with appropriate arguments
        call_descriptors, eval_function = parseCallString(func_string, mode,
                                                          self._makeHandlerMethodConfigAndGetExecutionFunction)
        # Return the resulting function to be evaluated
        return eval_function

    def instantiateAllHandlers(self):
        """
        instantiate all the handlers of the main robot of the current executing config
        instantiate only the init handler of the non-main robot
        """
        for robot in self.executing_config.robots:
            if robot.name == self.executing_config.main_robot:
                # this is a main robot
                for handler_type_class in ht.getAllHandlerTypeClass():
                    if handler_type_class in robot.handlers:
                        h = self.prepareHandler(robot.handlers[handler_type_class])
                    # if this is a init handler, set the shared_data
                    if handler_type_class == ht.InitHandler:
                        self.executor.proj.shared_data.update(h.getSharedData())
            else:
                # this is a non-main robot
                h = self.prepareHandler(robot.getHandlerOfRobot(ht.InitHandler))
                # this is a init handler, set the shared_data
                self.executor.proj.shared_data.update(h.getSharedData())


    def createHandlerMethodConfig(self, robot_name, handler_name, method_name, kwargs):
        """
        Create a handler method config object from the given information of this method
        """

        # find which handler does this method belongs to
        handler_config = None

        if robot_name == "share":
            # this ia a dummy sensor/actuator
            handler_type = None
            handler_config = self.getHandlerConfigDefault(robot_name, handler_type, handler_name)
        else:
            # this is a robot sensor/actuator
            for robot_config in self.executing_config.robots:
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

        # find the handler instance
        h = self.prepareHandler(method_config.handler)

        # Get the function object and make its arg dict
        method_config.method_reference = getattr(h, method_config.name)

        # update parameters of this method
        method_config.updateParaFromDict(kwargs)

        return method_config


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

        try:
            call_descriptors, _ = parseCallString(method_string, mode="single")
            if len(call_descriptors[0].name) != 3:
                raise SyntaxError("Name must be in form of '<robot_name>.<handler_name>.<method_name>'")
        except SyntaxError:
            logging.exception("Cannot parse setting {!r}".format(method_string))
            return None

        robot_name, handler_name, method_name = call_descriptors[0].name

        # find which handler does this method belongs to
        handler_config = None

        if robot_name == "share":
            # this is a dummy sensor/actuator
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
        method_config.updateParaFromDict(call_descriptors[0].args)

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
        para_info = ', '.join("{}={!r}".format(k, v)
                              for k, v in method_config.getArgDict().iteritems())

        return '.'.join([robot_name, handler_name, method_name])+'('+para_info+')'

    def getSensorValue(self, prop_name_list):
        """
        given a list of proposition names, return dictionary with {prop_name:sensor_value},
        where sensor_value is a boolean value returned by sensor handler
        """

        sensor_state = {}
        for prop_name in prop_name_list:
            if prop_name not in self.prop2func.keys():
                raise ValueError("Cannot find proposition {} in the given proposition mapping".format(prop_name))
            else:
                sensor_state[prop_name] = self.prop2func[prop_name](initial=False)

        return sensor_state

    def setActuatorValue(self, actuator_state):
        """
        given a dictionary with {prop_name:actuator_value},
        where actuator_value is a boolean value that gets passed to actuator handler
        """

        for prop_name, actuator_value in actuator_state.iteritems():
            if prop_name not in self.prop2func.keys():
                raise ValueError("Cannot find proposition {} in the given proposition mapping".format(prop_name))
            else:
                self.prop2func[prop_name](initial=False,
                                          actuatorVal=actuator_value)

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
    import execute
    e = execute.LTLMoPExecutor()
    filename = os.path.join(globalConfig.get_ltlmop_root(), "examples", "firefighting", "firefighting.spec")
    e.proj.loadProject(filename)
    e.hsub = HandlerSubsystem(e, e.proj.project_root)
    logging.info("Setting current executing config...")
    config, success = e.hsub.loadConfigFile(e.proj.current_config)
    if success: e.hsub.configs.append(config)
    e.hsub.setExecutingConfig(e.proj.current_config)

    logging.info("Importing handler functions...")
    e.hsub.prepareMapping()
    logging.info("Initializing all functions...")
    e.hsub.initializeAllMethods()

    while True:
        sensor_state = e.hsub.getSensorValue(e.proj.enabled_sensors)
        e.hsub.setActuatorValue({'pick_up': not sensor_state['person']})
        time.sleep(1)
