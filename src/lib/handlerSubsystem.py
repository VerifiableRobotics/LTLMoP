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

    def setValue(self,value,deli = ','):
        """
        This function makes sure all parameter are set according to the desired type

        The deli flag defines the deliminator used for list
        """

        if self.type.lower() == 'float':
            # A float type. Allows simple algebra and pi
            # TODO: supports more math
            value.replace("pi","3.1415")
            re.sub(r"[^\d\.\+-/\*]", "", value) # delete anything other than numbers or math operators
            try:
                self.value = float(eval(value))
            except SyntaxError:
                pass
        elif self.type.lower() == 'int' or self.type.lower() == 'integer':
            # A float type. Allows simple algebra
            re.sub(r"[^\d\.\+-/\*]", "", value) # delete anything other than numbers or math operators
            try:
                self.value = int(eval(value))
            except SyntaxError:
                pass
        elif self.type.lower() == 'bool' or self.type.lower() == 'boolean':
            if value.lower() in ['1','true','t']:
                self.value = True
            elif value.lower() in ['0','false','f']:
                self.value = False
        elif self.type.lower() == 'region':
            self.value = str(value).strip('\"\'')
        elif self.type.lower() == 'str' or self.type.lower() == 'string':
            self.value = str(value).strip('\"\'')
        elif self.type.lower() == 'listofint' or self.type.lower() == 'listofinteger':
            self.value = [int(eval(x)) for x in value.strip('\'\"[]').split(deli)]
        elif self.type.lower() == 'listoffloat':
            self.value = [float(eval(x)) for x in value.strip('\'\"[]').split(deli)]
        elif self.type.lower() == 'listofbool' or self.type.lower() == 'listofboolean':
            self.value = [bool(x) for x in value.strip('\'\"[]').split(deli)]
        elif self.type.lower() == 'listofstr' or self.type.lower() == 'listofstring':
            self.value = [str(x).strip('\"') for x in value.strip('\'\"[]').split(deli)]
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
        self.omitPara = []      # list of parameter names that are omitted

    def getParaByName(self, name):
        for p in self.para:
            if p.name == name:
                return p

        print "WARNING: Could not find parameter of name '%s' in method '%s'" % (name, self.name)
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

        print "WARNING: Could not find method of name '%s' in handler '%s'" % (name, self.name)
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
            fileName = '.'.join(['handlers.robots',robotFolder,self.name])
        else:
            fileName = '.'.join(['handlers',self.getType(),self.name])
        return fileName

    def getType(self):
        return self.type

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
    def __init__(self,proj):

        self.proj = proj
        self.silent = False
        self.handler_dic = None
        self.robots = None
        self.configs = None

        self.handler_path = os.path.join(self.proj.ltlmop_root,'lib','handlers')
        self.handler_parser = HandlerParser(self.handler_path)


        self.robot_parser = RobotFileParser(self.handler_path,self.handler_dic)

        self.config_path = os.path.join(self.proj.project_root,'configs')
        self.config_parser = ConfigFileParser(self.config_path,self.handler_path,self.proj,self.handler_dic)

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
        self.config_parser.setSilent(silent)

    def getRobotByType(self, t):
        for r in self.robots:
            if r.type == t:
                return r

        print "WARNING: Could not find robot of type '%s'" % (t)
        return None

    def getHandler(self, htype, hname, rname=None):
        if htype in self.handler_parser.handler_robotSpecific_type:
            if rname is None:
                if not self.silent: print "WARNING: Handler of type '%s' requires a robot type to be specified for lookup" % (htype)
                return None

            for h in self.handler_dic[htype][rname]:
                if h.name == hname:
                    return h
        else:
            for h in self.handler_dic[htype]:
                if h.name == hname:
                    return h

        if not self.silent: print "WARNING: Could not find handler of type '%s' with name '%s'" % (htype, hname)
        return None

    def string2Method(self,method_string):
        """
        Return the method object according to the input string
        """
        if self.handler_dic is None:
            print "ERROR: Cannot find handler dictionary, please load all handlers first."
            return

        method_info,para_info = method_string.split('(')
        para_info = [x.strip() for x in para_info.replace(')','').split(',')]

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
                            print "ERROR: Cannot recognize handler %s"  % handlerName
                            return

            if handlerObj is None:
                print "ERROR: Cannot recognize robot %s"  % robotName
                return

            for method_obj in handlerObj.methods:
                if method_obj.name == methodName:
                    methodObj = method_obj
                    break

            for para_pair in para_info:
                if '=' in para_pair:
                    para_name,para_value = [x.strip() for x in para_pair.split('=')]
                    for paraObj in methodObj.para:
                        if paraObj.name == para_name:
                            paraObj.setValue(para_value)
                            break
        else:
            print "ERROR: Cannot recognize method %s, please spicify which handler it belongs to." %method_info
            return

        return methodObj


    def method2String(self,methodObj,robotName=''):
        """
        Return the string representation according to the input method object
        """
        if self.handler_dic is None:
            print "ERROR: Cannot find handler dictionary, please load all handler first."
            return
        if not isinstance(methodObj,MethodObject):
            print "ERROR: Input is not a valid method object!"
            return
        if robotName=='':
            print "ERROR: Needs robot name for method2String"
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
                    if not self.silent: print "ERROR: Cannot recognize handler type %s"%handler_type

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
                if not self.silent: print "ERROR: Cannot find handler for %s" % handler_type


        # initiate all handlers
        for handler_type in all_handler_types:
            for robotName,handlerObj in self.h_obj[handler_type].iteritems():
                # get handler class object for initiating
                fileName = handlerObj.fullPath(robotName,configObj)
                if not self.silent: print "  -> %s" % fileName
                __import__(fileName)
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
                print "WARNING: No mapping given for sensor prop '%s', so using default simulated handler." % prop
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
                print "WARNING: No mapping given for actuator prop '%s', so using default simulated handler." % prop
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
                            paraObj.setValue(para_pair.split('=',1)[1],';')
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
        self.silent = True
        self.handler_path = path    # handler folder path
        self.handler_dic = {}       # dictionary for all handler information {type of handler:list of handlers of that type}
                                    # for sensor,actuator,init and locomotion handler the value is a dictionary {name of the robot: handler object}
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
                if not self.silent: print "(Handler Parser) WARNING: Cannot find %s handler folder..." % handler_type
            else:
                if handler_type == 'share':
                    self.handler_dic[handler_type] = self.loadHandler(handler_type,False)
                else:
                    self.handler_dic[handler_type] = self.loadHandler(handler_type,True)

        # now let's load all robot specific hanlders
        self.handler_dic['sensor'] = {'share': [handlerObj for handlerObj in self.handler_dic['share'] if 'sensor' in handlerObj.name.lower()]}
        self.handler_dic['actuator'] = {'share': [handlerObj for handlerObj in self.handler_dic['share'] if 'actuator' in handlerObj.name.lower()]}
        self.handler_dic['init'] = {}
        self.handler_dic['locomotionCommand'] = {}
        if 'robots' not in handlerFolders:
            if not self.silent: print "(Handler Parser) WARNING: Cannot find robot handler folder..."
        else:
            robotFolderList = os.listdir(os.path.join(self.handler_path,'robots'))
            for robotFolder in robotFolderList:
                if '.' not in robotFolder:
                    fileList = os.listdir(os.path.join(self.handler_path,'robots',robotFolder))
                    for fileName in fileList:
                        for handler_type in self.handler_robotSpecific_type:
                            if fileName.endswith('py') and (not fileName.startswith('_')) and handler_type.lower() in fileName.lower():
                                h_file = '.'.join(['handlers','robots',robotFolder,fileName.split('.')[0]])
                                if handler_type in ['init','locomotionCommand']:
                                    onlyLoadInit = True
                                else:
                                    onlyLoadInit = False
                                self.handler_dic[handler_type][robotFolder] = [self.parseHandlers(h_file,handler_type,onlyLoadInit)]

    def loadHandler(self,folder,onlyLoadInit=False):

        handlerList = []

        path = os.path.join(self.handler_path,folder)
        handlerFileList = os.listdir(path)

        for h_file in handlerFileList:
            if h_file.endswith('.py') and not h_file.startswith('_'):
                fileName = '.'.join(['handlers',folder,h_file.split('.')[0]])
                h_obj = self.parseHandlers(fileName,folder,onlyLoadInit)
                if h_obj is not None:
                    handlerList.append(h_obj)

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

        try:
            __import__(handlerFile)
        except ImportError:
            if not self.silent: print "WARNING: Failed to import handler %s" % handlerFile
            return None

        handlerModule = sys.modules[handlerFile]
        allClass = inspect.getmembers(handlerModule,inspect.isclass)
        for classObj in allClass:
            if classObj[1].__module__ == handlerFile and not classObj[0].startswith('_'):
                handlerClass = classObj[1]
                break

        # update the handler name and type info
        handlerObj.name = handlerFile.split('.')[-1]
        handlerObj.type = h_type

        # parse methods in this handler
        handlerMethod = inspect.getmembers(handlerClass,inspect.ismethod)
        for methodName,method in handlerMethod:
            if ((not onlyLoadInit and (not str(methodName).startswith('_')) or str(methodName)=='__init__') ):
                # load all parameters
                methodObj = MethodObject()
                methodObj.name = methodName
                methodObj.handler = handlerObj
                for para_name in inspect.getargspec(method)[0]:
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
                            for para in methodObj.para:
                                if para.name.lower() == m.group('argName').strip().lower():
                                    para.type = m.group('type')
                                    para.des = m.group('description')
                                    if m.group('range') is not None and numRE.search(m.group('range')):
                                        for pair in numRE.findall(m.group('range')):
                                            if pair[0] == 'default':
                                                para.default = pair[1]
                                                para.setValue(pair[1],';')
                                            elif pair[0] == 'min':
                                                para.min = pair[1]
                                            elif pair[0] == 'max':
                                                para.max = pair[1]
                                            else:
                                                print "Unrecognized argument comments \"%s\" for argument \"%s\" of method \"%s\"" %(pair[0],m.group('argName'),methodName)

                        # The line comments the function
                        else:
                            methodObj.comment += line + "\n"

                methodObj.comment = methodObj.comment.strip()

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
                        robotObj = self.loadRobotFile(os.path.join(self.handler_path,'robots',robotFolder,fileName))
                        if (robotObj is not None) and (robotObj.type not in [r.type for r in self.robots]):
                            self.robots.append(robotObj)

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

        try:
            # NOTE: If we cared about security, this would be a terrible idea
            mat_str = ''.join(robot_data['CalibrationMatrix'])
            if mat_str.strip() == "": raise KeyError()
        except KeyError:
            if not self.silent: print "WARNING: No calibration data found for robot '%s'" % robotObj.name
            robotObj.calibrationMatrix = None
        else:
            try:
                robotObj.calibrationMatrix = eval(mat_str)
            except SyntaxError:
                if not self.silent: print "WARNING: Invalid calibration data found for robot '%s'. Ignoring." % robotObj.name
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
                        print "ERROR: Wrong handler type %s in robot file %s" %(key,robotObj.type
+'.robot')
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
                        if handlerList[0] == None:
                            if not self.silent: print "WARNING: Cannot load all necessary handlers for robot %s" %robotObj.name
                            return None
                else:
                    if self.handler_dic is not None:
                        # we can just quick load from the handler dictionary
                        handlerList = self.handler_dic[handler_type]
                    else:
                        handlerParser = HandlerParser(self.handler_path)
                        fileName = '.'.join(['handlers',handler_type,val[0].split('(')[0]])
                        handlerList = [handlerParser.parseHandlers(fileName,handler_type,True)]

                # copy the handler object from the dictionary
                for handlerObj in handlerList:
                    if handlerObj is not None and handlerObj.name == val[0].split('(')[0]:
                        robotObj.handlers[handler_type] = deepcopy(handlerObj)

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
                                        paraObj.setValue(para_value,';')
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
        self.main_robot = '' # name of robot for moving in this config

    def getRobotByName(self, name):
        for r in self.robots:
            if r.name == name:
                return r

        if not self.silent: print "WARNING: Could not find robot of name '%s' in config '%s'." % (name, self.name)
        return None


class ConfigFileParser:
    """
    A parser loads all configuration files
    """

    def __init__(self,config_path,handler_path,proj,handler_dic=None):
        self.silent = False
        self.proj = proj
        self.config_path = config_path  # config folder path
        self.handler_path = handler_path    # handler folder path
        self.handler_dic = handler_dic  # handler dictionary stores all handler information
        self.configs = []   # list of config object

    def setSilent(self, silent):
        self.silent = silent

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

        if 'Initial_Truths' in config_data['General Config']:
            # parse the initially true propositions
            for propName in config_data['General Config']['Initial_Truths']:
                try:
                    configObj.initial_truths.append(propName)
                except IOError:
                    if not self.silent: print "ERROR: Wrong initially true propositions -- %s"


        try:
            configObj.main_robot = config_data['General Config']['Main_Robot'][0]
        except (IndexError, KeyError):
            if not self.silent: print "ERROR: Cannot find main robot for this config"

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
                    if robotObj is not None:
                        configObj.robots.append(robotObj)
                except IOError:
                    if not self.silent: print "ERROR: Cannot parse robot data in %s" % fileName

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
        return configObj

    def saveAllConfigFiles(self):
        for configObj in self.configs:
            self.saveConfigFile(configObj)

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
                    "Initial_Truths": "Initially true propositions"}

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

