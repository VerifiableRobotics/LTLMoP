#!/usr/bin/env python
"""
=========================================
dummyActuator.py - Dummy Actuator Handler
=========================================

Does nothing more than print the actuator name and state; for testing purposes.
"""

import subprocess, os, time, socket
import sys
import logging

class actuatorHandler:
    def __init__(self, proj, shared_data):
        self.proj = proj
        self.p_gui = None
        self.actuator_names = []

    def _stop(self):
        if self.p_gui is not None:
            print >>sys.__stderr__, "(SENS) Killing dummyactuator GUI..."
            try:
                self.p_gui.stdin.write(":QUIT\n")
                self.p_gui.stdin.close()
            except IOError:
                # Probably already closed by user
                pass

    def triggerResynthesis(self, actuatorVal, initial):
        """ Tell the executor to pause, resynthesize, and then resume. """
        # TODO: Move this to a separate shared handler

        # Only trigger on rising edges
        if not initial and int(actuatorVal) == 1:
            if self.proj.executor.needs_resynthesis:
                # We are in the middle of loading a new aut after resynthesis;
                # don't trigger in this case.
                return

            logging.debug("Resynthesis handler triggered")

            # We can't rely on being the last actuator that is called
            # for this state, so let's set a flag and let executor-resynthesis
            # handle things from here
            self.proj.executor.needs_resynthesis = True

    def setActuator(self, name, actuatorVal,initial):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (bool).
        
        name (string): Name of the actuator
        """

        if initial:
            # Start the GUI window if we haven't already
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
                print "(SENS) Starting actuatorHandler window..."
                self.p_gui = subprocess.Popen(["python", "-u", os.path.join(self.proj.ltlmop_root,"lib","handlers","share","_ActuatorHandler.py")], stderr=subprocess.PIPE, stdin=subprocess.PIPE)
                
                data = ''
                while "Hello!" not in data:
                    # Wait for and receive a message from the subwindow
                    try:
                        data,addrFrom = UDPSock.recvfrom(1024)
                    except socket.timeout:
                        print "Waiting for GUI..."
                        continue

                UDPSock.close()

            # Only initialize a new column for this prop name if we haven't
            # seen it before (for example, in the case of resynthesis)
            if name not in self.actuator_names:
                self.p_gui.stdin.write(name + ",init\n")
                self.actuator_names.append(name)
        else:
            if actuatorVal:
                time.sleep(0.1)  # Fake some time lag for the actuator to enable

            self.p_gui.stdin.write("{},{}\n".format(name,int(actuatorVal)))

            print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, actuatorVal)))

