#!/usr/bin/env python

""" ================================================
    handlerLoader.py - Load sensor/actuator handler file
    ================================================
    
    This module exposes an object that allows for simplified loading of the
    sensor and actuator hanlder files.
"""

import os, sys, re
import inspect
import lib.project as project

class varClass:
    """
    A variable object AoHwu!
    """

    def __init__(self,vname,vtype=None,vdes=None):
        self.name = vname
        self.type = vtype
        self.des = vdes
        self.default = None
        self.max = None
        self.min = None

class handlerLoader:
    """
    A handlerLoader object AoHwu!
    """

    def __init__(self,proj):
        self.silent = False
        self.proj = proj
        self.handlerPath = os.path.join(self.proj.ltlmop_root,'lib/handlers/')
        self.handlerMethods = []
        self.loadAllHandlers()

    def setSilent(self,silent):
        self.silent = silent

    def loadAllHandlers(self):
        """
        Load all handlers in the handler folder
        """

        handlerFolders = os.listdir(self.handlerPath)

        if 'pose' not in handlerFolders:
            print "WARNING: Cannot find pose handler folder..."        
        else:
            self.pose_handler = self.loadHandler('pose',True)
        if 'motionControl' not in handlerFolders:
            print "WARNING: Cannot find motion control handler folder..."        
        else:
            self.motion_handler = self.loadHandler('motionControl',True)
        if 'drive' not in handlerFolders:
            print "WARNING: Cannot find drive handler folder..."        
        else:
            self.drive_handler = self.loadHandler('drive',True)
        if 'share' not in handlerFolders:
            print "WARNING: Cannot find share handler folder..."        
        else:
            self.share_handler = self.loadHandler('share',True)
        if 'robots' not in handlerFolders:
            print "WARNING: Cannot find robot handler folder..."        
        else:
            self.robot_handler = {}
            robotFolderList = os.listdir(os.path.join(self.handlerPath,'robots'))
            for robotFolder in robotFolderList:
                if '.' not in robotFolder:
                    self.robot_handler[robotFolder] = self.loadHandler('robots/'+robotFolder)


    def loadHandler(self,folder,onlyLoadInit=False):
        handlerList = {}

        path = os.path.join(self.handlerPath,folder)
        handlerFileList = os.listdir(path)
        
        for h_file in handlerFileList:
            if h_file.endswith('.py') and not h_file.startswith('__'):
                handlerList[h_file.split('.')[0]] = self.importHandlers('lib.handlers.'+folder.replace('/','.')+'.'+h_file.split('.')[0],onlyLoadInit)
                map(self.handlerMethods.append,handlerList[h_file.split('.')[0]])

        return handlerList
    
    def importHandlers(self, handlerFile, onlyLoadInit=False):
        """
        Load method info (name,arg...) in the given handler file
        If onlyLoadInit is True, only the info of __init__ method will be loaded
        """

        # Regular expressions to help us out
        argRE = re.compile('(?P<argName>\w+)(\s*\((?P<type>\w+)\s*\))(\s*:\s*)(?P<description>[^\(]+)(\s*\((?P<range>.+)\s*\))?',re.IGNORECASE)
        #numRE = re.compile('default=(?P<default>\d*\.?\d+).+min=(?P<min>\d*\.?\d+).+max=(?P<max>\d*\.?\d+)',re.IGNORECASE)  
        numRE = re.compile('(?P<key>[^=]+)=(?P<val>[^,]+),?',re.IGNORECASE)

        #sys.path.append(self.proj.ltlmop_root)  # Temporary fix until paths get straightened out
        
        
        if not self.silent: print "  -> Loading %s " % handlerFile
        #handlerModule = __import__('%s' % handlerFile)
        
        try:
            __import__(handlerFile)
        except ImportError:
            print "WARNING: Error importing %s" % handlerFile
            return [] 

        handlerModule = sys.modules[handlerFile]
        allClass = inspect.getmembers(handlerModule,inspect.isclass)
        handlerClass = None
        for classObj in allClass:
            if classObj[1].__module__ == handlerFile and not classObj[0].startswith('__'):
                handlerClass = classObj[1]
                break

        if handlerClass is None:
            return []

        handlerMethod = inspect.getmembers(handlerClass,inspect.ismethod)

        handlerMethodDicList = []
        
        for methodName,method in handlerMethod:
            if (not onlyLoadInit) or str(methodName)=='__init__':
                MethodDic = {}
                MethodDic['handlerName'] = handlerClass.__module__.split('.')[-1]
                MethodDic['name'] = str(methodName)
                MethodDic['arg'] = []
                for arg in inspect.getargspec(method)[0]:
                    if arg not in ['self','initial','proj','shared_data']: 
                        var = varClass(arg)  
                        MethodDic['arg'].append(var)
                MethodDic['comment'] = ''
        
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
                            for var in MethodDic['arg']:
                                if var.name.lower() == m.group('argName').strip().lower():
                                    var.type = m.group('type')
                                    var.des = m.group('description')
                                    if m.group('range') is not None and numRE.search(m.group('range')):
                                        for pair in numRE.findall(m.group('range')):
                                            if pair[0] == 'default':
                                                var.default = pair[1]
                                            elif pair[0] == 'min':
                                                var.min = pair[1]
                                            elif pair[0] == 'max':
                                                var.max = pair[1] 
                                            else: 
                                                print "Unrecognized argument comments \"%s\" for argument \"%s\" of method \"%s\"" %(pair[0],m.group('argName'),methodName)               

                        # The line comments the function
                        else:
                            MethodDic['comment'] = MethodDic['comment']+(line)
                argToRemove = []        
                for i,var in enumerate(MethodDic['arg']):
                    if var.des == None:
                        argToRemove.append(i)
                            
#                print MethodDic['name']
#                print MethodDic['comment']
#                for var in MethodDic['arg']:
#                    print var.name
#                    print var.type
#                    print var.des
#                    if var.default is not None:
#                        print var.default
#                        print var.max
#                        print var.min   
                handlerMethodDicList.append(MethodDic)    

        return handlerMethodDicList               


    def getAvailableHandler(self,h_type,robot=None):
        """
        Return list of names of available handler in h_type
        
        h_type - name of type of handler: pose, drive, motionControl,robots
        """
        nameList = [] # list of names of of available handler in h_type
        
        
        if h_type == 'pose':
            return self.pose_handler.keys()
        if h_type == 'motion' or h_type == 'motionControl':
            h_list = []
            for handler in self.motion_handler.keys():
                if not handler.endswith('Helper'):
                    h_list.append(handler)
            return h_list
        if h_type == 'drive':
            return self.drive_handler.keys()
        if h_type == 'robot':
            return self.robot_handler.keys()
        if h_type == 'actuator':
            return self.robot_handler[robot].keys()
        if h_type == 'sensor':
            return self.robot_handler[robot].keys()
        if h_type == 'locomotionCommand':
            return self.robot_handler[robot].keys()
        if h_type == 'init':
            return self.robot_handler[robot].keys()
        print "ERROR: Cannot find handler type %s" % h_type
        return
       
    def getRobotHandler(self,robotName,h_type):
        """
        Return list of names of available method in h_type of given robot
        
        h_type - name of type of handler: sensor, actuator
        """    
        if robotName in self.robot_handler.keys():
            for handler in self.robot_handler[robotName].keys():
                if h_type in handler.lower():
                    return self.getMethodNames(self.robot_handler[robotName][handler])
            print "ERROR: Cannot find handler type %s" % h_type
            return
        else:
            print "ERROR: Cannot find robot %s" % robotName
            return
    
    def getMethodNames(self,methodDict):
        """
        Return list of method names of given method dictionary
        """
        methodNames = []
        for method in methodDict:
            if not method['name'].startswith('_'):
                methodNames.append(method['name'])
        return methodNames


    def printHandler(self):
    
        print
        print '========Handler for robot  ========'
        print
        for MethodDic in self.handlerMethods:
            print 'Method %s in %s' % (MethodDic['name'],MethodDic['handlerName'])
            print 'Method Description: %s' % MethodDic['comment']
            print
            if len(MethodDic['arg'])==0:
                print 'The method has no arguement.'
            else:
                print 'The method has following argurments...'
            print
            for var in MethodDic['arg']:
                if var.name not in ['self','initial']:
                    print '\tArgument Name: %s'%var.name
                if var.type is not None:
                    print '\tArgument Type: %s'%var.type
                if var.des is not None:
                    print '\tArgument Description: %s'%var.des
                if var.default is not None:
                    print '\tThe arguement has minimum value of %s, maximum value of %s and default value of %s.'%(var.min,var.max,var.default)
                print
            print '================================================='
            print

if __name__ == '__main__':
    proj = project.Project()
    proj.ltlmop_root = '/home/jim/Desktop/ltlmop_git/src'
    h = handlerLoader(proj)
    h.printHandler()
