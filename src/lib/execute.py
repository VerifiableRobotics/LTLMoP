#!/usr/bin/env python

""" =================================================
    execute.py - Top-level hybrid controller executor
    =================================================

    This module executes a hybrid controller for a robot in a simulated or real environment.

    :Usage: ``execute.py [-hn] [-p listen_port] [-a automaton_file] [-s spec_file]``

    * The controlling automaton is imported from the specified ``automaton_file``.

    * The supporting handler modules (e.g. sensor, actuator, motion control, simulation environment initialization, etc)
      are loaded according to the settings in the config file specified as current in the ``spec_file``.

    * If no port to listen on is specified, an open one will be chosen randomly.
    * Unless otherwise specified with the ``-n`` or ``--no_gui`` option, a status/control window
      will also be opened for informational purposes.
"""

import sys, os, getopt, textwrap
import threading, subprocess, time
import fsa, project
from copy import deepcopy
from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
import xmlrpclib
import socket
import random
import math
import traceback
from resynthesis import ExecutorResynthesisExtensions
import globalConfig, logging


####################
# HELPER FUNCTIONS #
####################

def usage(script_name):
    """ Print command-line usage information. """

    print textwrap.dedent("""\
                              Usage: %s [-hn] [-p listen_port] [-a automaton_file] [-s spec_file]

                              -h, --help:
                                  Display this message
                              -n, --no-gui:
                                  Do not show status/control window
                              -p PORT, --xmlrpc-listen-port PORT:
                                  Listen on PORT for XML-RPC calls
                              -a FILE, --aut-file FILE:
                                  Load automaton from FILE
                              -s FILE, --spec-file FILE:
                                  Load experiment configuration from FILE """ % script_name)

class LTLMoPExecutor(object, ExecutorResynthesisExtensions):
    """
    This is the main execution object, which combines the synthesized discrete automaton
    with a set of handlers (as specified in a .config file) to create and run a hybrid controller
    """

    def __init__(self):
        """
        Create a new execution context object
        """

        self.proj = project.Project() # this is the project that we are currently using to execute
        self.aut = None

        # Choose a timer func with maximum accuracy for given platform
        if sys.platform in ['win32', 'cygwin']:
            self.timer_func = time.clock
        else:
            self.timer_func = time.time

        self.externalEventTarget = None
        self.externalEventTargetRegistered = threading.Event()
        self.postEventLock = threading.Lock()
        self.runFSA = threading.Event()  # Start out paused
        self.alive = threading.Event()
        self.alive.set()

    def postEvent(self, eventType, eventData=None):
        """ Send a notice that an event occurred, if anyone wants it """
        
        with self.postEventLock:
            if self.externalEventTarget is None:
                return

            try:
                self.externalEventTarget.handleEvent(eventType, eventData)
            except socket.error as e:
                logging.warning("Could not send event to remote event target: %s", e)
                logging.warning("Forcefully unsubscribing target.")
                self.externalEventTarget = None

    def loadSpecFile(self, filename):
        # Update with this new project
        self.proj = project.Project()
        self.proj.loadProject(filename)

        # Tell GUI to load the spec file
        self.postEvent("SPEC", self.proj.getFilenamePrefix() + ".spec")

    def loadAutFile(self, filename):
        print "Loading automaton..."

        aut = fsa.Automaton(self.proj)

        success = aut.loadFile(filename, self.proj.enabled_sensors, self.proj.enabled_actuators, self.proj.all_customs + self.proj.internal_props)

        return aut if success else None

    def _getCurrentRegionFromPose(self, rfi=None):
        # TODO: move this to regions.py
        if rfi is None:
            rfi = self.proj.rfi

        pose = self.proj.coordmap_lab2map(self.proj.h_instance['pose'].getPose())

        region = next((i for i, r in enumerate(rfi.regions) if r.name.lower() != "boundary" and \
                        r.objectContainsPoint(*pose)), None)
 
        if region is None:
            print "Pose of ", pose, "not inside any region!"

        return region

    def shutdown(self):
        self.runFSA.clear()
        print >>sys.__stderr__, "QUITTING."

        all_handler_types = ['init', 'pose', 'locomotionCommand', 'drive', 'motionControl', 'sensor', 'actuator']

        for htype in all_handler_types:
            print >>sys.__stderr__, "terminating {} handler...".format(htype)
            if htype in self.proj.h_instance:
                if isinstance(self.proj.h_instance[htype], dict):
                    handlers = [v for k,v in self.proj.h_instance[htype].iteritems()]
                else:
                    handlers = [self.proj.h_instance[htype]]
            
                for h in handlers:
                    if hasattr(h, "_stop"):
                        print >>sys.__stderr__, "> calling _stop() on {}".format(h.__class__.__name__) 
                        h._stop()
                    else:
                        print >>sys.__stderr__, "> {} does not have _stop() function".format(h.__class__.__name__) 
            else:
                print >>sys.__stderr__, "not found in h_instance"

        self.alive.clear()

    def pause(self):
        """ pause execution of the automaton """
        self.runFSA.clear()
        time.sleep(0.1) # Wait for FSA to stop
        self.postEvent("PAUSE")

    def resume(self):
        """ start/resume execution of the automaton """
        self.runFSA.set()

    def isRunning(self):
        """ return whether the automaton is currently executing """
        return self.runFSA.isSet()

    def getCurrentGoalNumber(self):
        """ Return the index of the goal currently being pursued (jx).
            If no automaton is loaded, return None. """
        if not self.aut:
            return None
        else:
            return self.aut.next_state.rank

    def registerExternalEventTarget(self, address):
        self.externalEventTarget = xmlrpclib.ServerProxy(address, allow_none=True)

        # Redirect all output to the log
        redir = RedirectText(self.externalEventTarget.handleEvent)

        sys.stdout = redir
        sys.stderr = redir

        self.externalEventTargetRegistered.set()

    def initialize(self, spec_file, aut_file, firstRun=True):
        """
        Prepare for execution, by loading and initializing all the relevant files (specification, map, handlers, aut)

        If `firstRun` is true, all handlers will be imported; otherwise, only the motion control handler will be reloaded.
        """

        # load project only first time; otherwise self.proj is modified in-place
        # TODO: make this less hacky
        if firstRun:
            self.loadSpecFile(spec_file)

            if self.proj.compile_options['decompose']:
                self.proj.rfiold = self.proj.rfi  # Save the undecomposed regions

        if self.proj.compile_options['decompose']:
            self.proj.rfi = self.proj.loadRegionFile(decomposed=True)

        if self.proj.currentConfig is None:
            print "ERROR: Can not simulate without a simulation configuration."
            print "Please create one by going to [Run] > [Configure Simulation...] in SpecEditor and then try again."
            sys.exit(2)

        # Import the relevant handlers
        if firstRun:
            print "Importing handler functions..."
            self.proj.importHandlers()
        else:
            #print "Reloading motion control handler..."
            #self.proj.importHandlers(['motionControl'])
            pass

        # We are done initializing at this point if there is no aut file yet
        if aut_file is None:
            return

        # Load automaton file
        new_aut = self.loadAutFile(aut_file)

        if firstRun:
            ### Wait for the initial start command
            print "Ready.  Press [Start] to begin..."
            self.runFSA.wait()

        ### Figure out where we should start from

        init_region = self._getCurrentRegionFromPose()
        if init_region is None:
            print "Initial pose not inside any region!"
            sys.exit(-1)

        print "Starting from initial region: " + self.proj.rfi.regions[init_region].name

        ### Have the FSA find a valid initial state

        if firstRun or self.aut is None:
            # Figure out our initially true outputs
            init_outputs = []
            for prop in self.proj.currentConfig.initial_truths:
                if prop not in self.proj.enabled_sensors:
                    init_outputs.append(prop)

            init_state = new_aut.chooseInitialState(init_region, init_outputs)
        else:
            # Figure out our initially true outputs
            init_outputs = [k for k,v in self.aut.current_outputs.iteritems() if int(v) == 1]

            init_state = new_aut.chooseInitialState(init_region, init_outputs)#, goal=prev_z)

        if init_state is None:
            print "No suitable initial state found; unable to execute. Quitting..."
            sys.exit(-1)
        else:
            print "Starting from state %s." % init_state.name

        self.aut = new_aut

    def run(self):
        ### Get everything moving
        # Rate limiting is approximately 20Hz
        avg_freq = 20
        last_gui_update_time = 0

        # FIXME: don't crash if no spec file is loaded initially
        while self.alive.isSet():
            # Idle if we're not running
            if not self.runFSA.isSet():
                try:
                    self.proj.h_instance['motionControl'].stop()
                except AttributeError:
                    self.proj.h_instance['drive'].setVelocity(0,0)

                # wait for either the FSA to unpause or for termination
                while (not self.runFSA.wait(0.1)) and self.alive.isSet():
                    pass

            # Exit immediately if we're quitting
            if not self.alive.isSet():
                break
            
            self.prev_outputs = deepcopy(self.aut.current_outputs)
            self.prev_z = self.aut.current_state.rank

            tic = self.timer_func()
            self.aut.runIteration()
            toc = self.timer_func()

            #self.checkForInternalFlags()

            # Rate limiting of execution and GUI update
            while (toc - tic) < 0.05:
                time.sleep(0.005)
                toc = self.timer_func()

            # Update GUI
            # If rate limiting is disabled in the future add in rate limiting here for the GUI:
            # if show_gui and (timer_func() - last_gui_update_time > 0.05)
            avg_freq = 0.9 * avg_freq + 0.1 * 1 / (toc - tic) # IIR filter
            self.postEvent("FREQ", int(math.ceil(avg_freq)))
            pose = self.proj.h_instance['pose'].getPose(cached=True)[0:2]
            self.postEvent("POSE", tuple(map(int, self.proj.coordmap_lab2map(pose))))

            last_gui_update_time = self.timer_func()

        print "execute.py quitting..."

    # This function is necessary to prevent xmlrpcserver from catching
    # exceptions and eating the tracebacks
    def _dispatch(self, method, args): 
        try: 
            return getattr(self, method)(*args) 
        except: 
            traceback.print_exc()
            raise

class RedirectText:
    def __init__(self, event_handler):
        self.event_handler = event_handler

    def write(self, message):
        if message.strip() != "":
            self.event_handler("OTHER", message.strip())

    def flush(self):
        pass


####################################################
# Main function, run when called from command-line #
####################################################

def execute_main(listen_port=None, spec_file=None, aut_file=None, show_gui=False):
    print "\n[ LTLMOP HYBRID CONTROLLER EXECUTION MODULE ]\n"
    print "Hello. Let's do this!\n"

    # Create the XML-RPC server
    if listen_port is None:
        # Search for a port we can successfully bind to
        while True:
            listen_port = random.randint(10000, 65535)
            try:
                xmlrpc_server = SimpleXMLRPCServer(("127.0.0.1", listen_port), logRequests=False, allow_none=True)
            except socket.error as e:
                pass
            else:
                break
    else:
        xmlrpc_server = SimpleXMLRPCServer(("127.0.0.1", listen_port), logRequests=False, allow_none=True)
    
    # Create the execution context object
    e = LTLMoPExecutor()

    # Register functions with the XML-RPC server
    xmlrpc_server.register_instance(e)

    # Kick off the XML-RPC server thread    
    XMLRPCServerThread = threading.Thread(target=xmlrpc_server.serve_forever)
    XMLRPCServerThread.daemon = True
    XMLRPCServerThread.start()
    print "Executor listening for XML-RPC calls on http://127.0.0.1:{} ...".format(listen_port)

    # Start the GUI if necessary
    if show_gui:
        # Create a subprocess
        print "Starting GUI window..."
        p_gui = subprocess.Popen(["python", "-u", os.path.join(project.get_ltlmop_root(), "lib", "simGUI.py"), str(listen_port)])

        # Wait for GUI to fully load, to make sure that
        # to make sure all messages are redirected
        e.externalEventTargetRegistered.wait()

    if spec_file is not None:
        # Tell executor to load spec & aut
        #if aut_file is None:
        #    aut_file = spec_file.rpartition('.')[0] + ".aut"
        e.initialize(spec_file, aut_file, firstRun=True)

    # Start the executor's main loop in this thread
    e.run()

    # Clean up on exit
    print "Waiting for XML-RPC server to shut down..."
    xmlrpc_server.shutdown()
    XMLRPCServerThread.join()


### Command-line argument parsing ###

if __name__ == "__main__":
    ### Check command-line arguments

    aut_file = None
    spec_file = None
    show_gui = True
    listen_port = None

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hnp:a:s:", ["help", "no-gui", "xmlrpc-listen-port=", "aut-file=", "spec-file="])
    except getopt.GetoptError, err:
        print str(err)
        usage(sys.argv[0])
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage(sys.argv[0])
            sys.exit()
        elif opt in ("-n", "--no-gui"):
            show_gui = False
        elif opt in ("-p", "--xmlrpc-listen-port"):
            try:
                listen_port = int(arg)
            except ValueError:
                print "ERROR: Invalid port '{}'".format(arg)
                sys.exit(2)
        elif opt in ("-a", "--aut-file"):
            aut_file = arg
        elif opt in ("-s", "--spec-file"):
            spec_file = arg

    execute_main(listen_port, spec_file, aut_file, show_gui)
