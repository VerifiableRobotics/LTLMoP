#!/usr/bin/env python
"""
=========================================
dummyActuator.py - Dummy Actuator Handler
=========================================

Does nothing more than print the actuator name and state; for testing purposes.
"""

import subprocess, os, time, socket
import sys

import lib.handlers.handlerTemplates as handlerTemplates

class DummyActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):
        self.proj = executor.proj
        self.p_gui = None

    def _stop(self):
        if self.p_gui is not None:
            print >>sys.__stderr__, "(ACT) Killing dummyactuator GUI..."
            try:
                self.p_gui.stdin.write(":QUIT\n")
                self.p_gui.stdin.close()
            except IOError:
                # Probably already closed by user
                pass

    def setActuator(self, name, actuatorVal,initial):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (bool).

        name (string): Name of the actuator
        """

        if initial:
            if self.p_gui is None:
                # Prepare to receive initialization signal
                host = 'localhost'
                port = 23559
                buf = 1024
                addr = (host,port)

                UDPSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                UDPSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    UDPSock.bind(addr)
                except:
                    print "ERROR: Cannot bind to port.  Try killing all Python processes and trying again."
                    return

                # Create a subprocess
                print "(ACT) Starting actuatorHandler window..."
                self.p_gui = subprocess.Popen([sys.executable, "-u", os.path.join(self.proj.ltlmop_root,"lib","handlers","share","Actuator","_ActuatorHandler.py")], stderr=subprocess.PIPE, stdin=subprocess.PIPE)

                data = ''
                while "Hello!" not in data:
                    # Wait for and receive a message from the subwindow
                    try:
                        data,addrFrom = UDPSock.recvfrom(1024)
                    except socket.timeout:
                        print "Waiting for GUI..."
                        continue

                UDPSock.close()

            self.p_gui.stdin.write(name + ",init\n")
        else:
            if actuatorVal:
                time.sleep(0.1)  # Fake some time lag for the actuator to enable

            self.p_gui.stdin.write("{},{}\n".format(name,int(actuatorVal)))

            print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, actuatorVal)))

