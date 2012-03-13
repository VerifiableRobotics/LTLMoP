#!/usr/bin/env python


""" ================================================
    configClass.py - Object from a .config file
    ================================================
    
    This module exposes an object that allows for loading/editing/saving the data 
    in a given .config file
"""


import os, sys, re
import fileMethods, regions
import inspect,types
from numpy import *
from copy import deepcopy
import project  


class ParameterObject:
    """
    A parameter object!
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
        if self.type.lower() == 'float':
            self.value = float(value)
        elif self.type.lower() == 'int' or self.type.lower() == 'integer':
            self.value = int(value)
        elif self.type.lower() == 'bool' or self.type.lower() == 'boolean':
            self.value = bool(eval(value))
        elif self.type.lower() == 'str' or self.type.lower() == 'string':
            self.value = str(value)
        elif self.type.lower() == 'listofint' or self.type.lower() == 'listofinteger':
            self.value = [int(x) for x in eval(value)]
        elif self.type.lower() == 'listoffloat':
            self.value = [float(x) for x in eval(value)]
        elif self.type.lower() == 'listofbool' or self.type.lower() == 'listofboolean':
            self.value = [bool(x) for x in eval(value)]
        elif self.type.lower() == 'listofstr' or self.type.lower() == 'listofstring':
            self.value = [str(x) for x in eval(value)]
        else:
            print 'ERROR: Invalid parameter type %s' % self.type

    def getValue(self):
        return self.value


class MethodObject:
    """
    A method object in handler file
    """
    def __init__(self):
        self.name = None        # name of the method
        self.handler = None     # which handler the method belongs to
        self.comment = ''       # comment of the method 
        self.para = []          # list of parameter object of this method
        self.reference = None   # reference of this method in the memory

class HandlerObject:
    """
    A handler object!
    """
    def __init__(self):
        self.name = None    # name of the handler
        self.type = None    # type of the handler e.g. motionControl or drive
        self.methods = []   # list of method objects in this handler 

    def toString(self):
        """
        Return the string representation of the handler object
        """
        

class RobotObject:
    """
    A Robot object
    """
    def __init__(self,r_name=None,r_type=None,driveH=None,initH=None,locoH=None,motionH=None,poseH=None,sensorH=None,actuatorH=None):
        self.name = r_name  # name of the robot
        self.type = r_type  # type of the robot
        self.handlers = {'drive':driveH, 'init':initH, 'locomotionCommand':locoH, 'motionControl':motionH, 'pose':poseH, 'sensor':sensorH,'actuator':actuatorH} # dictionary of handler object for this robot

class HandlerSubsystem:
    """
    Interface dealing with configuration files and hanlders    
    """
    def __init__(self,proj):

        self.proj = proj
        self.silent = False
        self.handler_dic = None
        self.robots = None
        self.configs = None

        self.handler_path = os.path.join(self.proj.ltlmop_root,'lib','handlers')
        self.handler_parser = HandlerParser(self.handler_path)
        #self.handler_parser.printHandler()

        
        #testStringBefore = 'share.dummySensor.buttonPress(sensor_name="Fire")'
        #testMethod = self.string2Method(testStringBefore)
        #print testMethod.name,testMethod.handler,testMethod.para[0].type

        #testStringAfter = self.method2String(testMethod)
        #print testStringBefore == testStringAfter


        self.robot_parser = RobotFileParser(self.handler_path,self.handler_dic)
         
        self.config_path = os.path.join(self.proj.project_root,'configs')
        self.config_parser = ConfigFileParser(self.config_path,self.handler_path,self.handler_dic)

        #print self.configs[0].robots[0].handlers['init:nao'].methods[0].para[1].value
        #print self.robots[0].name

        #print "Done!"
    
    def loadAllHandlers(self):
        self.handler_parser.loadAllHandlers()
        self.handler_dic = self.handler_parser.handler_dic

    def loadAllRobots(self):
        self.robot_parser.loadAllRobots()
        self.robots = self.robot_parser.robots

    def loadAllConfigFiles(self):
        self.config_parser.loadAllConfigFiles()
        self.configs = self.config_parser.configs


    def setSilent(self,silent):
        self.silent = silent

    def string2Method(self,method_string):
        """
        Return the method object according to the input string
        """
        if self.handler_dic is None:
            print "ERROR: Cannot find handler dictionary, please load all handler first."
            return

        method_info,para_info = method_string.split('(')
        para_info = [x.strip() for x in para_info.replace(')','').split(',')]

        items = method_info.split('.')
        methodObj = MethodObject()

        if len(items) == 3:
            robotName = items[0]
            handlerName = items[1]
            methodName = items[2]

            handlerList = []

            if robotName == 'share':
                # this ia a dummy sensor/actuator
                for h in self.handler_dic['share']:
                    if h.name == handlerName:
                        handlerList = [h]
                        break
                if len(handlerList)==0:
                    print "ERROR: Cannot recognize handler %s"  % handlerName
                    return
            else:
                # this ia a robot sensor/actuator
                if 'sensor' in handlerName.lower():
                    handlerList = self.handler_dic['sensor'][robotName]
                    
                elif 'actuator' in handlerName.lower():
                    handlerList = self.handler_dic['actuator'][robotName]

                else:
                    print "ERROR: Cannot recognize handler %s"  % handlerName
                    return

            for handlerObj in handlerList:
                if handlerObj.name == handlerName:
                    for method_obj in handlerObj.methods:
                        if method_obj.name == methodName:
                            methodObj = method_obj

            for para_pair in para_info:
                para_name,para_value = [x.strip() for x in para_pair.split('=')]
                for paraObj in methodObj.para:
                    if paraObj.name == para_name:
                        paraObj.setValue(para_value)
                        break
        else:
            print "ERROR: Cannot recognize method %s, please spicify which handler it belongs to." %method_info
            return

        return methodObj


    def method2String(self,methodObj):
        """
        Return the string representation according to the input method object 
        """
        if self.handler_dic is None:
            print "ERROR: Cannot find handler dictionary, please load all handler first."
            return
        if not isinstance(methodObj,MethodObject):
            print "ERROR: Input is not a valid method object!"
            return

        handlerType = ''
        handlerName = methodObj.handler
        methodName = methodObj.name

        if 'dummy' in handlerName:
            handlerType = 'share'
        elif 'sensor' in handlerName:
            handlerType = 'sensor'
        elif 'actuator' in handlerName:
            handlerType = 'actuator'
        else:
            print "ERROR: Invalid handler Type %s of method %s!" %(handlerName,methodName)
            return

        # convert all parameter object into string
        para_list = []
        for paraObj in methodObj.para:
            if paraObj.value is None:
                para_list.append( paraObj.name+'='+str(paraObj.default))
            else:
                para_list.append( paraObj.name+'='+str(paraObj.value))
        para_info = ','.join(para_list)

        return '.'.join([handlerType,handlerName,methodName])+'('+para_info+')'
        



class HandlerParser:
    """
    A parser loads all handler information    
    """
    def __init__(self,path):
        self.silent = True
        self.handler_path = path    # handler folder path
        self.handler_dic = {}       # dictionary for all handler information {type of handler:list of handlers of that type} 
                                    # for sensor,actuator,init and locomotion handler the value is a dictionary {name of the robot: handler object}
        self.handler_types = ['pose','drive','motionControl','share']       # list of types of handlers, must match with the folder name in lib/handler folder
        self.handler_robotSpecific_type = ['init','locomotionCommand','sensor','actuator']   # list of types of robot specific handlers in each robot folder
        self.ignore_parameter = ['self','initial','proj','shared_data']     # list of name of parameter that should be ignored where parse the handler methods


    def loadAllHandlers(self):
        """
        Load all handlers in the handler folder
        """

        handlerFolders = os.listdir(self.handler_path)

        # load all robot-independent handlers first
        for handler_type in self.handler_types:
            self.handler_dic[handler_type] = []
            if handler_type not in handlerFolders:
                if self.silent: print "(Handler Parsor) WARNING: Cannot find %s handler folder..." % handler_type
            else:
                if handler_type == 'share':
                    self.handler_dic[handler_type] = self.loadHandler(handler_type,False)
                else:
                    self.handler_dic[handler_type] = self.loadHandler(handler_type,True)
        
        # now let's load all robot specific hanlders
        self.handler_dic['sensor'] = {}
        self.handler_dic['actuator'] = {}
        self.handler_dic['init'] = {}
        self.handler_dic['locomotionCommand'] = {}
        if 'robots' not in handlerFolders:
            print "(Handler Parsor) WARNING: Cannot find robot handler folder..."       
        else:
            robotFolderList = os.listdir(os.path.join(self.handler_path,'robots'))
            for robotFolder in robotFolderList:
                if '.' not in robotFolder:
                    fileList = os.listdir(os.path.join(self.handler_path,'robots',robotFolder))
                    for fileName in fileList:
                        for handler_type in self.handler_robotSpecific_type:
                            if fileName.endswith('py') and (not fileName.startswith('__')) and handler_type.lower() in fileName.lower():
                                h_file = os.path.join('lib','handlers','robots',robotFolder,fileName.split('.')[0]).replace('/','.')
                                if handler_type in ['init','locomotionCommand']:
                                    onlyLoadInit = True
                                else:
                                    onlyLoadInit = False
                                self.handler_dic[handler_type][robotFolder] = [self.parseHandlers(h_file,handler_type+':'+robotFolder,onlyLoadInit)]

    def loadHandler(self,folder,onlyLoadInit=False):

        handlerList = []

        path = os.path.join(self.handler_path,folder)
        handlerFileList = os.listdir(path)
        
        for h_file in handlerFileList:
            if h_file.endswith('.py') and not h_file.startswith('__'):
                fileName = os.path.join('lib','handlers',folder,h_file.split('.')[0]).replace('/','.')
                handlerList.append(self.parseHandlers(fileName,folder,onlyLoadInit))

        return handlerList
    
    def parseHandlers(self,handlerFile,h_type,onlyLoadInit=False):
        """
        Load method info (name,arg...) in the given handler file
        If onlyLoadInit is True, only the info of __init__ method will be loaded

        returns a handler object
        """
        
        handlerObj = HandlerObject()

        # Regular expressions to help us out
        argRE = re.compile('(?P<argName>\w+)(\s*\((?P<type>\w+)\s*\))(\s*:\s*)(?P<description>[^\(]+)(\s*\((?P<range>.+)\s*\))?',re.IGNORECASE)
        numRE = re.compile('(?P<key>[^=]+)=(?P<val>[^,]+),?',re.IGNORECASE)
        
        # start to load the handler file
        if not self.silent: print "  -> Loading %s " % handlerFile
        
#        sys.path.append(os.path.join(self.proj.ltlmop_root, 'lib')
        __import__(handlerFile)
        handlerModule = sys.modules[handlerFile]
        allClass = inspect.getmembers(handlerModule,inspect.isclass)
        for classObj in allClass:
            if classObj[1].__module__ == handlerFile and not classObj[0].startswith('__'):
                handlerClass = classObj[1]
                break

        # update the handler name and type info
        handlerObj.name = handlerFile.split('.')[-1]
        handlerObj.type = h_type

        # parse methods in this handler
        handlerMethod = inspect.getmembers(handlerClass,inspect.ismethod)
        for methodName,method in handlerMethod:
            if (not onlyLoadInit) or str(methodName)=='__init__':
                # load all parameters
                methodObj = MethodObject()
                methodObj.name = methodName
                methodObj.handler = handlerObj.name
                for para_name in inspect.getargspec(method)[0]:
                    if para_name not in self.ignore_parameter: 
                        paraObj = ParameterObject(para_name)  
                        methodObj.para.append(paraObj)
        
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
                            for para in methodObj.para:
                                if para.name.lower() == m.group('argName').strip().lower():
                                    para.type = m.group('type')
                                    para.des = m.group('description')
                                    if m.group('range') is not None and numRE.search(m.group('range')):
                                        for pair in numRE.findall(m.group('range')):
                                            if pair[0] == 'default':
                                                para.default = pair[1]
                                                para.setValue(pair[1])
                                            elif pair[0] == 'min':
                                                para.min = pair[1]
                                            elif pair[0] == 'max':
                                                para.max = pair[1] 
                                            else: 
                                                print "Unrecognized argument comments \"%s\" for argument \"%s\" of method \"%s\"" %(pair[0],m.group('argName'),methodName)               

                        # The line comments the function
                        else:
                            methodObj.comment = methodObj.comment+(line)


                # if there are parameter that has no description
                argToRemove = []  
                for para in methodObj.para:
                    if para.des == None:
                        argToRemove.append(para)
                map(methodObj.para.remove,argToRemove)

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
        self.silent = True
        self.handler_types = ['pose','drive','motionControl','share']       # list of types of handlers, must match with the folder name in lib/handler folder
        self.handler_robotSpecific_type = ['init','locomotionCommand','sensor','actuator']   # list of types of robot specific handlers in each robot folder
        self.ignore_parameter = ['self','initial','proj','shared_data']     # list of name of parameter that should be ignored where parse the handler methods

        self.handler_dic = handler_dic  # dictionary stores all handler information for faster reference
        self.handler_path = path        # handler folder path


    def setSilent(self, silent):
        self.silent = silent
    
    def loadAllRobots(self):
        robotFolderList = os.listdir(os.path.join(self.handler_path,'robots'))
        for robotFolder in robotFolderList:
            if '.' not in robotFolder:
                fileList = os.listdir(os.path.join(self.handler_path,'robots',robotFolder))
                for fileName in fileList:
                    if fileName.endswith('.robot'):
                        self.robots.append(self.loadRobotFile(os.path.join(self.handler_path,'robots',robotFolder,fileName)))

    def loadRobotFile(self,fileName):
    
        # Add extension to the name if there isn't one. 
        if not fileName.endswith('.robot'):
            fileName = fileName+'.robot'  
     
        if not self.silent: print "Loading robot file %s..." % fileName
        try:
            # try to load the robot file
            robot_data = fileMethods.readFromFile(fileName)
        except IOError: 
            if not self.silent: print "ERROR: Cannot find robot file %s" % fileName        
            return

        return self.loadRobotData(robot_data)

    def loadRobotData(self,robot_data):

        robotObj = RobotObject(r_name=robot_data['RobotName'][0],r_type=robot_data['Type'][0])      
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
                            handler_type = h_type+':'+robotObj.type
                            break
                    if handler_type == None:
                        print "ERROR: Wrong handler type %s in robot file %s" %(key,robotObj.type
+'.robot')
                        return
                    else:
                        if self.handler_dic is not None:
                            # we can just quick load from the handler dictionary
                            handlerList = self.handler_dic[handler_type.split(':')[0]][robotFolder]
                        else:
                            handlerParser = HandlerParser(self.handler_path)
                            fileName = os.path.join('lib','handlers','robots',robotFolder,val[0].split('(')[0]).replace('/','.')
                            handlerList = [handlerParser.parseHandlers(fileName,handler_type,True)]
                else:
                    if self.handler_dic is not None:
                        # we can just quick load from the handler dictionary
                        handlerList = self.handler_dic[handler_type]
                    else:
                        handlerParser = HandlerParser(self.handler_path)
                        fileName = os.path.join('lib','handlers',handler_type,val[0].split('(')[0]).replace('/','.')
                        handlerList = [handlerParser.parseHandlers(fileName,handler_type,True)]
                
                # copy the handler object from the dictionary
                for handlerObj in handlerList:
                    if handlerObj.name == val[0].split('(')[0]:
                        
                        robotObj.handlers[handler_type] = deepcopy(handlerObj)
                        break
                # overwrite the parameter values
                para_list = [x.strip() for x in val[0].split('(')[1].replace(')','').split(',')]

                for methodObj in robotObj.handlers[handler_type].methods:
                    if methodObj.name == '__init__':
                        initMethodObj = methodObj
                        break
                for para in para_list:
                    if '=' in para:
                        para_name,para_value = [x.strip() for x in para.split('=')]
                        
                        for paraObj in initMethodObj.para:
                            if para_name == paraObj.name:
                                paraObj.setValue(para_value)
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


class ConfigFileParser:
    """
    A parser loads all configuration files    
    """

    def __init__(self,config_path,handler_path,handler_dic=None):
        self.silent = False
        self.config_path = config_path  # config folder path
        self.handler_path = handler_path    # handler folder path
        self.handler_dic = handler_dic  # handler dictionary stores all handler information
        self.configs = []   # list of config object

    def setSilent(self, silent):
        self.silent = silent

    def loadAllConfigFiles(self):
        
        fileList = os.listdir(self.config_path)
        for fileName in fileList:
            if fileName.endswith('.config'):
                configObj = self.loadConfigFile(os.path.join(self.config_path,fileName))
                if configObj is not None:
                    self.configs.append(configObj)
                
    def loadConfigFile(self,fileName):
        # Add extension to the name if there isn't one. 
        if not fileName.endswith('.config'):
            fileName = fileName+'.config'  
     
        if not self.silent: print "Loading config file %s..." % fileName
        try:
            # First try path relative to project path
            config_data = fileMethods.readFromFile(fileName)
        except IOError: 
            if not self.silent: print "ERROR: Cannot find config file %s" % fileName        
            return
    
        configObj = ConfigObject()
        try:
            configObj.name = config_data['General Config']['Name'][0]
            
        except IOError: 
            if not self.silent: print "ERROR: Missing general config information in config file %s" % fileName
        
        # parse the string for sensor prop mapping
        for sensorMapping in config_data['General Config']['Sensor_Proposition_Mapping']:
            try:
                sensorProp,sensorFun = [s.strip() for s in sensorMapping.split('=',1)]
            except IOError: 
                if not self.silent: print "ERROR: Wrong sensor mapping -- %s" % sensorMapping        

            configObj.prop_mapping[sensorProp]=sensorFun

        # parse the string for actuator prop mapping
        for actuatorMapping in config_data['General Config']['Actuator_Proposition_Mapping']:
            try:
                actuatorProp,actuatorFun = [s.strip() for s in actuatorMapping.split('=',1)]
            except IOError: 
                if not self.silent: print "ERROR: Wrong actuator mapping -- %s" % actuatorMapping        
            configObj.prop_mapping[actuatorProp]=actuatorFun

        
        # load robot configs
        robot_data = []
        for configKey,configValue in config_data.iteritems():
            if configKey.startswith('Robot'):
                robot_data.append(configValue)

        if robot_data == []:
            if not self.silent: print "ERROR: Missing Robot data in config file %s" % fileName

        else:
            # using the parsing function in RobotFileParser to parse the data
            robot_parser = RobotFileParser(self.handler_path,self.handler_dic)
            for data in robot_data:
                try:
                    robotObj = robot_parser.loadRobotData(data)
                    configObj.robots.append(robotObj)
                except IOError: 
                    if not self.silent: print "ERROR: Cannot parse robot data in %s" % fileName        

        return configObj

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
            if 'sensor' in fun:
                sensorMapping = prop + ' = ' + fun
                sensorMappingList.append(sensorMapping)
            elif 'actuator' in fun:
                actuatorMapping = fun + ' = ' + fun
                actuatorMappingList.append(actuatorMapping)

        data['General Config']['Sensor_Proposition_Mapping'] = sensorMappingList
        data['General Config']['Actuator_Proposition_Mapping'] = actuatorMappingList
                
        for i,robot in enumerate(configObj.robots):
            header = 'Robot'+str(i+1)+' Config'
            data[header]={}
            data[header]['RobotName'] = robot.name
            data[header]['Type'] = robot.type
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
                    "Actuator_Proposition_Mapping": 'Mapping between actuator propositions and actuator handler functions',
                    "Sensor_Proposition_Mapping": "Mapping between sensor propositions and sensor handler functions",
                    "Name": 'Configuration name'}

        fileMethods.writeToFile(os.path.join(self.proj.project_root,'configs',fileName), data, comments)

                   

if __name__ == '__main__':
    proj = project.Project()
    proj.ltlmop_root = '/home/jim/Desktop/ltlmop_git/src'
    proj.project_root = '/home/jim/Desktop/ltlmop_git/src/examples/newSensorTest'
    h = HandlerSubsystem(proj)
