#!/usr/bin/env python
"""
=================================================
PioneerRealInit.py - Real Pioneer Robot Initialization Handler
=================================================

Initialize the sockets to communicate with C# on the Pioneer
"""

import time
from socket import *
from struct import pack, unpack
from threading import Thread, Lock, Event
import Polygon,Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes
import matplotlib.pyplot as plt
from numpy import *

class PioneerRealInitHandler:

    def __init__(self, proj,LocalIP,ListenerPort,BroadcasterIP,BroadcasterPort):
        """
        Init Handler for pioneer real robot.

        LocalIP (string)        : Local IP of Pioneer           (default='0.0.0.0')
        ListenerPort (int)      : Listering Port of Pioneer     (default=1000)
        BroadcasterIP (string)  : Broadcasting IP of the lab    (default='0.0.0.0')
        BroadcasterPort (int)   : Broadcasting Port of the lab  (default=5000)
        """


        # Get connection settings from robot configuration file
        ipIn = LocalIP                  # IP address (string)
        portIn = ListenerPort           # Port (number)
        ipOut = BroadcasterIP           # IP address (string)
        portOut = BroadcasterPort       # Port (number)
        try:
            # Create proxies to access modules
            self.robocomm = _RobotCommunicator()
            self.robocomm.start()
            time.sleep(1)   # Give communicator time to start and receive first data
        except RuntimeError:
            print "(INIT) ERROR: Cannot connect to the robot."
            exit(-1)

    def getSharedData(self):
        # Return dictionary of module proxies for other handlers to use
        return {'robocomm':self.robocomm}

class _RobotCommunicator:
    """
    Class used to communicate from LTLMoP in Python to C# on Pioneers or
    Segways. Is designed to be consistent with LTLMoPSerializer C# class.
    """

    # Static message headers
    POSE_HEADER = '\xAA'
    SENSOR_HEADER = '\xBB'
    WAYPOINT_HEADER = '\xCC'
    DIRECTION_HEADER = '\xDD'
    ACTUATOR_HEADER = '\xEE'
    WALL_HEADER = '\x11'
    OBS_HEADER  = '\x22'

    # Message formats (excluding leading byte)
    POSE_FORMAT = 'ddddddd' # (x,y,z,yaw,pitch,roll,timestamp)
    SENSOR_FORMAT = 'B?'    # (index,value)
                            # 'B' will become the byte specifying sensor index
    WAYPOINT_FORMAT = 'dd'  # (x,y)
                            # Constitutes a single waypoint which may repeat
    DIRECTION_FORMAT = 'dd' # (x,y)
    ACTUATOR_FORMAT = 'B'   # (index)
                            # 'B' will become the byte specifying actuator index
                            # Constitutes a single actuator which may repeat
    WALL_FORMAT = 'dddd'    # (index,x1,y1,x2,y2)
                            # Constitutes a single wall which may repeat
                            # 'B' will become the byte specifying wall index
    OBS_FORMAT  = 'ddd'     # (index, x1 ,y1)
                            # index =  1 --> add point
                            # index = -1 --> delete point
                            # index =  4 --> emergency stop requests
                            #            --> follow by 1, Don't have to stop Pioneer
                            #            --> follow by 0, stop Pioneer

    # Communication parameters
    #LOCAL_IP = "0.0.0.0"       # boardcasting local IP
    #LOCAL_IP = "10.0.0.122"    # Catherine's computer
    #LOCAL_IP = "10.0.0.107"    # BEE09
    LOCAL_IP = "10.0.0.190"     # BEE06
    DEFAULT_LISTEN_PORT = 6501
    #NETWORK_BROADCAST_IP = "10.255.255.255" # lab boardcasting
    NETWORK_BROADCAST_IP = "10.0.0.96"       # spider06 (Pioneer)

    DEFAULT_BROADCAST_PORT = 6502
    DEFAULT_BUFFER_SIZE = 10240

    # Constructor
    def __init__(self):
        # Communication parameters
        self.listenerIP = _RobotCommunicator.LOCAL_IP
        self.listenerPort = _RobotCommunicator.DEFAULT_LISTEN_PORT
        self.broadcasterIP = _RobotCommunicator.NETWORK_BROADCAST_IP
        self.broadcasterPort = _RobotCommunicator.DEFAULT_BROADCAST_PORT
        self.buffer = _RobotCommunicator.DEFAULT_BUFFER_SIZE

        # Communication threads
        self.listener = _RobotListener(self.listenerIP,self.listenerPort,
                                      self.buffer)
        self.broadcaster = RobotBroadcaster(self.broadcasterIP,
                                            self.broadcasterPort,self.buffer)


    def setIPAddress(self,listenIP,listenPort,broadcastIP,broadcastPort):
        """
        Set the IP addresses and ports of the listener and broadcaster.
        A value of None for any argument sets the field to its default value.
        IP address arguments should be in string format (see below).
        Values will not be changed if a socket is currently in use.

        Default values:
          listenIP = "0.0.0.0"  # local computer
          listenPort = 6501
          broadcastIP = "10.0.0.255"  # network broadcast address
          broadcastPort = 6502
        """
        # Listener
        if self.listener.close.isSet():
            if listenIP:
                self.listenerIP = listenIP
            else:
                self.listenerIP = _RobotCommunicator.LOCAL_IP
            if listenPort:
                self.listenerPort = listenPort
            else:
                self.listenerPort = _RobotCommunicator.DEFAULT_LISTEN_PORT
            self.listener = _RobotListener(self.listenerIP,self.listenerPort,
                                          self.buffer)

        # Broadcaster
        if broadcastIP:
            self.broadcasterIP = broadcastIP
        else:
            self.broadcasterIP = _RobotCommunicator.NETWORK_BROADCAST_IP
        if broadcastPort:
            self.broadcasterPort = broadcastPort
        else:
            self.broadcasterPort = _RobotCommunicator.DEFAULT_BROADCAST_PORT
        self.broadcaster = RobotBroadcaster(self.broadcasterIP,
                                            self.broadcasterPort,self.buffer)

    def start(self):
        """
        Open sockets for communication.
        """
        self.listener.start()
        # No need to start broadcaster, it just sends when necessary

    def stop(self):
        """
        Close sockets and prevent further communication.
        """
        self.listener.stop()
        self.broadcaster.stop()

    # Send data through broadcaster
    def sendPose(self,pose):
        """
        Serialize and send the pose to the robot.

        pose is a tuple containing 7 doubles
        (x,y,z,yaw,pitch,roll,timestamp)
        """
        self.broadcaster.sendPose(pose)

    def sendSensors(self,sensors):
        """
        Serialize and send the sensor readings to the robot.

        sensors is a dictionary containing byte keys and boolean values.
        {index : value, ...}

        index (the byte key) must be sent as a character.
        value (the boolean value) need only evaluate to the correct truth-value
        eg. broadcaster.sendSensors({'\x01' : 1, '\x0B' : 0})
            would send two sensor readings: sensor 1 = true, sensor 11 = false
        """
        self.broadcaster.sendSensors(sensors)

    def sendWaypoints(self,waypoints):
        """
        Serialize and send the waypoint data to the robot.

        waypoints is a list of tuples, each containing 2 doubles.
        [(x,y), ...]
        """
        self.broadcaster.sendWaypoints(waypoints)

    def sendDirection(self,direction):
        """
        Serialize and send the direction vector command to the robot.
        Also used to send velocity commands to the robot.

        direction is a tuple containing 2 doubles.
        (x,y)
        or (v,w) depending on application.
        """
        self.broadcaster.sendDirection(direction)

    def sendActuators(self,actuators):
        """
        Serialize and send the actuator activation commands to the robot.

        actuators is a list containing single bytes.
        [index, ...]

        index must be sent as a character (since Python doesn't have bytes).
        eg. broadcaster.sendActuators('\x02\x12\x0A')
            would make actuators 2, 18, and 10 activate
        """
        self.broadcaster.sendActuators(actuators)

    def sendWalls(self,walls):
        """
        Serialize and send the wall position data to the robot.

        walls is a dictionary containing byte keys and tuples containing 4
        doubles as values.
        {index : (x1,y1,x2,y2), ...}
        """
        self.broadcaster.sendWalls(walls)

    # Get data from listener
    def getPose(self):
        """
        Extract the latest pose data.

        Returns tuple with 7 doubles.
        (x,y,z,yaw,pitch,roll,timestamp)
        """
        return self.listener.pose

    def getSensors(self):
        """
        Extract the latest sensor data for every sensor with a value so far.

        Returns dictionary with byte sensor indices as keys and boolean sensor
        readings as values. Each index is actually a character with the value of
        the index.
        {index : value, ...}
        """
        return self.listener.sensors

    def getWaypoints(self):
        """
        Extract the latest waypoint data sent.

        Returns a list of tuples with 2 doubles each.
        [(x,y), ...]
        """
        return self.listener.waypoints

    def getDirection(self):
        """
        Extract the latest direction vector command sent.

        Returns a tuple with 2 doubles.
        (x,y)
        """
        return self.listener.direction

    def getActuators(self):
        """
        Extracts the latest actuator commands sent.

        Returns a list with a single byte indicating each actuator index.
        [index, ...]
        """
        return self.listener.actuators

    def getWalls(self):
        """
        Extracts the latest wall information sent.

        Returns a dictionary with byte wall indices as keys and tuples with 4
        doubles as values. Each index is actually a character with the value
        of the index.
        {index : (x1,y1,x2,y2), ...}
        """
        #print 'TYPE CHECK!!!!!!',self.listener.walls
        return self.listener.walls

    def clearWalls(self):
        #print "cleaning wall"
        self.listener.walls = (0,0,0,0)

    def getObs(self):
        """
        Extracts the latest obs information sent.

        Returns a dictionary with byte wall indices as keys and tuples with 4
        doubles as values. Each index is actually a character with the value
        of the index.
        {[x2,y2]}
        """
        return self.listener.obs

    def getAllObs(self):
        """
        Extracts the latest obs information sent.

        Returns array of all obstacles points from occupancy grid
        [[x1,y1],[x2,y2] ...]
        """
        #print 'getAllOBS:',self.listener.allObs
        return self.listener.allObs

    def getAddObs(self):
        """
        Extracts the latest obs information sent.

        Returns array of obstacles to be added to the map
        [[x1,y1],[x2,y2] ...]
        """
        #print 'getAddOBS:',self.listener.addObs
        return self.listener.addObs

    def getDelObs(self):
        """
        Extracts the latest obs information sent.

        Returns array of obstacles to be deleted from the map
        [[x1,y1],[x2,y2] ...]
        """
        #print 'getDelOBS:',self.listener.delObs
        return self.listener.delObs

    def getObsPoly(self):
        """
        Extracts Obstacle Polygon

        Returns array of Obstacles Polygon points [[x1,y1],[x2,y2] ...]

        """
        #print 'getObsPoly:',self.listener.obsPoly
        return self.listener.obsPoly

    def getReceiveObs(self):

        """
        Extracts the flag that first obstacle info from Pioneer has been received

        Return
        True  : obstacle info recevied
        False : obstacle info was never sent to ltlmop
        """

        return self.listener.receiveObs

    def getSTOP(self):

        """
        Emergency stopping flag when there is obstacle right next to Pioneer

        Returns

        True : if there is an obstacle within 0.35m of Pioneer
        False: there is no obstacle nearby
        """
        return self.listener.STOP

class _RobotListener(Thread):
    """
    Class used to communicate from a robot to LTLMoP. Is designed to be
    consistent with LTLMoPSerializer C# class.
    """

    # Constructor
    def __init__(self,ipAddress,port,bufferSize):
        # Superclass constructor
        super(_RobotListener,self).__init__()

        # Communication parameters
        self.addr = (ipAddress,port)
        self.buffer = bufferSize
        self.udpSock = socket(AF_INET,SOCK_DGRAM)
        self.lock = Lock()
        self.close = Event()

        #build self.obsPoly with empty contour
        self.receiveObs = False    # state of first obstacle data from ltlmop (start with false  )
        self.obsPoly = Polygon.Polygon()     #Polygon built from the occupancy grid
        self.resolX  = 0.1 * 1.05;        #Blow Up by 5 % grid width
        self.resolY  = 0.1 * 1.05;        #Blow Up by 5 % grid height
        self.STOP    = False              #emergency stop when there are obstacles right next to it
        self.POLTOBS = True

        # Data fields
        self.pose = ()      # Tuple of doubles (x,y,z,yaw,pitch,roll,timestamp)
        self.sensors = {}   # Dictionary byte:boolean {index : value, ...}
                            # The index will be a char since Python has no byte
                            # The value has unknown type, only truth value known
        self.waypoints = () # List of tuples of doubles [(x,y), ...]
        self.direction = () # Tuple of doubles (x,y)
        self.actuators = () # List of bytes [index, ...]
                            # Each index will be a char since Python has no byte
        self.walls = (0,0,0,0)     # Dictionary byte:tuple of doubles
                            # {index : (x1,y1,x2,y2), ...}
                            # Each index will be a char since Python has no byte
        self.allObs = [] # contains data of all obstacles
        self.obs = []          # contains latest obstacle data
        self.addObs = [] # adding data to the map
        self.delObs = [] # deleting data in the map

    # Start communication and receive messages
    def run(self):  #CHANGED FROM run to start
        """
        Open the socket to start UDP communication.
        Receive and process messages.
        """

        # Open socket for communication
        self.udpSock.bind(self.addr)
        # Receive communication until stopped
        while not self.close.isSet():
            data = self.udpSock.recv(self.buffer)
            self.lock.acquire()
            self.processData(data)
            self.lock.release()


        # Close socket
        self.udpSock.close()

    # Stop communication
    def stop(self):
        """
        Close the socket to end UDP communication.
        """
        self.close.set()

    # Deserialize and save data
    def processData(self,data):
        """
        Save the data to the appropriate category for possible extraction.
        """
        #print 'I GOT DATA',data,[0],data[1]
        # Check for valid data (not null or empty string)
        #print '**************NOTIFICATION***************',type(_RobotCommunicator.WALL_HEADER),type(data[0])
        if data:
            #print '**************NOTIFICATION***************',type(_RobotCommunicator.WALL_HEADER),type(data[0]),_RobotCommunicator.WALL_HEADER==data[0]

            # Check header and assign data appropriately
            # TODO: Check length of data for validity
            #print 'Header',data[0]
            if data[0] == _RobotCommunicator.POSE_HEADER:
                self.pose = unpack(_RobotCommunicator.POSE_FORMAT,data[1:])
            elif data[0] == _RobotCommunicator.SENSOR_HEADER:

                #for i in range(1, len(data)-1, 2):
                index= unpack('B',data[1])
                value = unpack('?',data[2])
                    # Update old values or create new sensor-value pair
                self.sensors[index[0]] = value[0]
                #print 'in csharp: ',[index,value]

            elif data[0] == _RobotCommunicator.WAYPOINT_HEADER:
                self.waypoints = [] # Clear old waypoints
                for i in range(1, len(data)-16, 16):
                    x,y = unpack(_RobotCommunicator.WAYPOINT_FORMAT,
                                 data[i:i+15])
                    self.waypoints.append((x,y))
            elif data[0] == _RobotCommunicator.DIRECTION_HEADER:
                self.direction = unpack(_RobotCommunicator.DIRECTION_FORMAT,
                                        data[1:])
            elif data[0] == _RobotCommunicator.ACTUATOR_HEADER:
                self.actuators = [] # Clear old actuator commands                for i in range(1, len(data)-1):
                self.actuators.append(unpack(
                        _RobotCommunicator.ACTUATOR_FORMAT,data[i]))
            elif data[0] == _RobotCommunicator.WALL_HEADER:
                self.walls = {} # Clear old wall entries
                index = unpack('B', data[1])
                x1,y1,x2,y2 = unpack(_RobotCommunicator.WALL_FORMAT,data[2:34])
                self.walls = (x1,y1,x2,y2)
                #print '**************Coordinates***************',(x1,y1,x2,y2)
                print '****self.walls*********',self.walls
            elif data[0] == _RobotCommunicator.OBS_HEADER:
                index = unpack('B', data[1])
                add,x1,y1 = unpack(_RobotCommunicator.OBS_FORMAT,data[2:26])
                #print '***********self.obs*************'+','.join(map(str,[add,x1,y1]))
                self.obs = [add,x1,round(y1,2)]
                if add == 1:
                    a = PolyShapes.Rectangle(self.resolX,self.resolY)
                    a.shift(x1,y1)
                    self.obsPoly += a
                    self.receiveObs = True
                    #print "add obstacle:" + str(x1) + ","+ str(y1)
                elif add == 4:
                    if x1 == 0:
                        self.STOP = True
                    else:
                        self.STOP = False
                else:
                    a = PolyShapes.Rectangle(self.resolX,self.resolY)
                    a.shift(x1,y1)
                    self.obsPoly -= a
                    self.receiveObs = True
                    #print "del obstacle:"+ str(x1) + ","+ str(y1)


            else:
                print "Unexpected or corrupted data packet received."


class RobotBroadcaster:
    """
    Clas
    ent with LTLMoPSerializer C# class.
    """

    # Constructor
    def __init__(self,ipAddress,port,bufferSize):
        # Communication parameters
        self.addr = (ipAddress,port)
        self.udpSock = socket(AF_INET,SOCK_DGRAM)
        so_broadcast = True;
        self.udpSock.setsockopt(SOL_SOCKET,SO_BROADCAST,so_broadcast)


    # Disable communication
    def stop(self):
        """
        Close the socket to end UDP communication.
        """
        self.udpSock.close()

    # Serialize and send data
    def sendPose(self,pose):
        """
        Serialize and send the pose to the robot.

        pose is a tuple containing 7 doubles
        (x,y,z,yaw,pitch,roll,timestamp)
        """
        x,y,z,yaw,pitch,roll,ts = pose
        data = _RobotCommunicator.POSE_HEADER + \
               pack(_RobotCommunicator.POSE_FORMAT,x,y,z,yaw,pitch,roll,ts)
        self.udpSock.sendto(data,self.addr)

    def sendSensors(self,sensors):
        """
        Serialize and send the sensor readings to the robot.

        sensors is a dictionary containing byte keys and boolean values.
        {index : value, ...}

        index (the byte key) must be sent as a character.
        value (the boolean value) need only evaluate to the correct truth-value
        eg. broadcaster.sendSensors({'\x01' : 1, '\x0B' : 0})
            would send two sensor readings: sensor 1 = true, sensor 11 = false
        """
        data = _RobotCommunicator.SENSOR_HEADER
        for index in sensors:
            data = data + pack(_RobotCommunicator.SENSOR_FORMAT,
                               index,sensors[index])
        self.udpSock.sendto(data,self.addr)

    def sendWaypoints(self,waypoints):
        """
        Serialize and send the waypoint data to the robot.

        waypoints is a list of tuples, each containing 2 doubles.
        [(x,y), ...]
        """
        data = _RobotCommunicator.WAYPOINT_HEADER
        for waypoint in waypoints:
            x,y = waypoint
            data = data + pack(_RobotCommunicator.WAYPOINT_FORMAT,x,y)
        self.udpSock.sendto(data,self.addr)

    def sendDirection(self,direction):
        """
        Serialize and send the direction vector command to the robot.
        Also used to send velocity commands to the robot.

        direction is a tuple containing 2 doubles.
        (x,y)
        or (v,w) depending on application.
        """
        x,y = direction
        data = _RobotCommunicator.DIRECTION_HEADER + \
               pack(_RobotCommunicator.DIRECTION_FORMAT,x,y)
        self.udpSock.sendto(data,self.addr)

    def sendActuators(self,actuators):
        """
        Serialize and send the actuator activation commands to the robot.

        actuators is a list containing single bytes.
        [index, ...]

        index must be sent as a character (since Python doesn't have bytes).
        eg. broadcaster.sendActuators('\x02\x12\x0A')
            would make actuators 2, 18, and 10 activate
        """
        data = _RobotCommunicator.ACTUATOR_HEADER
        for actuator in actuators:
            data = data + pack(_RobotCommunicator.ACTUATOR_FORMAT,actuator)
        self.udpSock.sendto(data,self.addr)

    def sendWalls(self,walls):
        """
        Serialize and send the wall position data to the robot.

        walls is a dictionary containing byte keys and tuples containing 4
        doubles as values.
        {index : (x1,y1,x2,y2), ...}

        index must be sent as a character (since Python doesn't have bytes).
        eg. broadcaster.sendWalls({'\x0A' : (1,3,2,4)})
            would indicate wall 10 has endpoints (1,3) and (2,4)
        """
        data = _RobotCommunicator.WALL_HEADER
        for index in walls:
            x1,y1,x2,y2 = walls[index]
            data = data + pack(_RobotCommunicator.WALL_FORMAT,
                               index,x1,y1,x2,y2)
        self.udpSock.sendto(data,self.addr)
