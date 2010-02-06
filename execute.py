#!/usr/bin/env python

""" =================================================
    execute.py - Top-level hybrid controller executor
    =================================================
    
    This module executes a hybrid controller for a robot in a simulated or real environment.

    :Usage: ``execute.py [-hn] [-a automaton_file] [-s spec_file]``

    * The controlling automaton is imported from the specified ``automaton_file``.

    * The supporting handler modules (e.g. sensor, actuator, motion control, simulation environment initialization, etc)
      are loaded according to the settings provided in the specified ``spec_file``.

    * Unless otherwise specified with the ``-n`` or ``--no_gui`` option, a status/control window
      will also be opened for informational purposes.
"""

import sys, os, getopt, textwrap
import threading, subprocess, time
import fileMethods, regions, fsa
from numpy import *

####################
# HELPER FUNCTIONS #
####################

def usage(script_name):
    """ Print command-line usage information. """

    print textwrap.dedent("""\
                              Usage: %s [-hn] [-a automaton_file] [-s spec_file]

                              -h, --help:
                                  Display this message
                              -n, --no-gui:
                                  Do not show status/control window
                              -a FILE, --aut-file FILE:
                                  Load automaton from FILE
                              -s FILE, --spec-file FILE:
                                  Load experiment configuration from FILE """ % script_name)

####################
# THREAD FUNCTIONS #
####################

def guiListen():
    """
    Processes messages from the GUI window, and reacts accordingly
    """

    global guiListenInitialized, runFSA

    while 1: 
        # Wait for and receive a message from the subwindow
        input = fd_gui_output.readline()
        if input == '':  # EOF indicates that the connection has been destroyed
            print "GUI listen thread is shutting down!"
            break

        input = input.strip()

        # Check for the initialization signal, if necessary
        if not guiListenInitialized and input == "Hello!":
            guiListenInitialized = True
            continue

        # Do what the GUI tells us, lest we anger it
        if input == "START":
            runFSA = True
        elif input == "PAUSE":
            runFSA = False
            print "\nPAUSED."
        else:
            print "WARNING: Unknown command received from GUI: " + input

#########################
# MAIN EXECUTION THREAD #
#########################

def main(argv):
    """ Main function; run automatically when called from command-line """

    ################################
    # Check command-line arguments #
    ################################

    aut_file = None
    spec_file = None
    show_gui = True

    try:
        opts, args = getopt.getopt(argv[1:], "hna:s:", ["help", "no-gui", "aut-file=", "spec-file="])
    except getopt.GetoptError, err:
        print str(err)
        usage(argv[0])
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage(argv[0])
            sys.exit()
        elif opt in ("-n", "--no-gui"):
            show_gui = False
        elif opt in ("-a", "--aut-file"):
            aut_file = arg
        elif opt in ("-s", "--spec-file"):
            spec_file = arg
    
    if aut_file is None:
        print "ERROR: Automaton file needs to be specified."
        usage(argv[0])
        sys.exit(2)

    if spec_file is None:
        print "ERROR: Specification file needs to be specified."
        usage(argv[0])
        sys.exit(2)

    print "\n[ LTLMOP HYBRID CONTROLLER EXECUTION MODULE ]\n"
    print "Hello. Let's do this!\n"

    ############################
    # Load configuration files #
    ############################

    # Figure out where we should be looking for files, based on the spec file name & location
    project_root = os.path.dirname(spec_file)
    project_basename, ext = os.path.splitext(os.path.basename(spec_file)) 
    ltlmop_root = os.path.dirname(sys.argv[0])

    #print "Project basename: %s" % project_basename
    #print "Relative project root path: %s" % project_root

    # Load in the specification file
    print "Loading specification file %s..." % spec_file
    spec_data = fileMethods.readFromFile(spec_file)   

    # Figure out the name of the current experiment config
    exp_cfg_name = spec_data['SETTINGS']['CurrentExperimentConfig'][0]

    # Find the details for this configuration
    key = "EXPERIMENT CONFIG: " + exp_cfg_name
    if key in spec_data:
        exp_cfg_data = spec_data[key]
        print "  -> Using experiment configuration \"%s\"" % exp_cfg_name
    else:
        print "ERROR: Could not find section \"%s\" in spec file!" % key
        sys.exit(0)
    
    #### Load in the lab setup file

    lab_name = exp_cfg_data['Lab'][0]
 
    print "Loading lab setup file %s..." % lab_name
    try:
        # First try path relative to project path
        lab_data = fileMethods.readFromFile(os.path.join(project_root, lab_name))   
    except IOError: 
        try:
            # If that doesn't work, try looking in $ltlmop_root/labs/ directory
            lab_data = fileMethods.readFromFile(os.path.join(ltlmop_root, "labs", lab_name))   
        except IOError:
            print "ERROR: Couldn't find lab setup file in project directory or labs folder."
            sys.exit(1)
    print "  -> Looks like you want to run your experiment with %s. Good choice." % lab_data["Name"][0]

    #### Load in the robot file
    
    rdf_name = exp_cfg_data['RobotFile'][0]
 
    print "Loading robot description file %s..." % rdf_name
    try:
        # First try path relative to project path
        rdf_data = fileMethods.readFromFile(os.path.join(project_root, rdf_name))   
    except IOError: 
        try:
            # If that doesn't work, try looking in $ltlmop_root/robots/ directory
            rdf_data = fileMethods.readFromFile(os.path.join(ltlmop_root, "robots", rdf_name))   
        except IOError:
            print "ERROR: Couldn't find robot description file in project directory or robots folder."
            sys.exit(1)
    print "  -> %s looks excited for this run." % rdf_data["Name"][0]

    #### Load in the region file

    regf_name = spec_data['SETTINGS']['RegionFile'][0]

    print "Loading region file %s..." % regf_name
    rfi = regions.RegionFileInterface() 
    rfi.readFile(os.path.join(project_root, regf_name))
 
    print "  -> Found definitions for %d regions." % len(rfi.regions)

    #### Create the coordmap functions

    # Look for transformation values in spec file
    try:
        transformValues = exp_cfg_data['Calibration'][0].split(",")
        [xscale, xoffset, yscale, yoffset] = map(float, transformValues)
    except KeyError, ValueError:
        print "Error: Please calibrate and update values before running simulation."
        sys.exit(0)

    # Create functions for coordinate transformation
    # (numpy may seem like overkill for this, but we already have it as a dependency anyways...)
    scale = diag([xscale, yscale])
    inv_scale = linalg.inv(scale)
    offset = array([xoffset, yoffset])
    fwd_coordmap = lambda pt: (dot(scale, pt) + offset)
    rev_coordmap = lambda pt: (dot(inv_scale, pt - offset))

    ##########################
    # Initialize each module #
    ##########################

    # Figure out which handlers we are going to use
    # TODO: Complain nicely instead of just dying when this breaks?
    h_name = {}
    h_name['init'] = lab_data["InitializationHandler"]
    h_name['pose'] = lab_data["PoseHandler"][0]
    h_name['sensor'] = lab_data["SensorHandler"][0]
    h_name['actuator'] = lab_data["ActuatorHandler"][0]
    h_name['locomotionCommand'] = lab_data["LocomotionCommandHandler"][0]
    h_name['motionControl'] = rdf_data["MotionControlHandler"][0]
    h_name['drive'] = rdf_data["DriveHandler"][0]

    # Import the relevant handlers
    print "Importing handler functions..."
    
    # We treat initialization handlers separately, because there may be multiple ones
    # NOTE: These will be loaded in the same order as they are listed in the config file

    # TODO: handle calibration somewhere around here? maybe?

    shared_data = {}  # This is for storing things like server connection objects, etc.
    init_num = 1
    init_handlers = []
    for handler in h_name['init']:
        print "  -> %s" % handler
        # TODO: Is there a more elegant way to do this? This is pretty ugly...
        exec("from %s import initHandler as initHandler%d" % (handler, init_num)) in locals() # WARNING: This assumes our input data is not malicious...
        exec("init_handlers.append(initHandler%d(project_root, project_basename, exp_cfg_data, rdf_data, fwd_coordmap, rfi, calib=False))" % (init_num)) in locals()
        shared_data.update(init_handlers[-1].getSharedData())
        init_num += 1  # So they don't clobber each other
        
    # Now do the rest of them
    for handler in ['pose','sensor','actuator','locomotionCommand','drive','motionControl']:
        print "  -> %s" % h_name[handler]
        exec("from %s import %sHandler" % (h_name[handler], handler)) in globals() # WARNING: This assumes our input data is not malicious...
        if handler == 'pose':
            pose_handler = poseHandler(shared_data)
        elif handler == 'sensor':
            # Figure out what sensors are enabled, and which are initially true
            all_sensors = []
            initial_sensors = []
            for line in spec_data['SETTINGS']['Sensors']:
                sensor, val = line.split(',')
                if int(val) == 1: 
                    all_sensors.append(sensor)
                    if sensor in exp_cfg_data['InitialTruths']:
                        initial_sensors.append(sensor)

            sensor_handler = sensorHandler(shared_data, all_sensors, initial_sensors)
        elif handler == 'actuator':
            actuator_handler = actuatorHandler(shared_data)
        elif handler == 'locomotionCommand':
            loco_handler = locomotionCommandHandler(shared_data)
        elif handler == 'drive':
            drive_handler = driveHandler(shared_data, loco_handler)
        elif handler == 'motionControl':
            motion_handler = motionControlHandler(shared_data, pose_handler, drive_handler, rfi, fwd_coordmap)


    #######################
    # Load automaton file #
    #######################
    
    FSA = fsa.Automaton(rfi.regions, sensor_handler, actuator_handler, motion_handler)

    # Figure out what actuators are enabled
    all_actuators = []
    for line in spec_data['SETTINGS']['Actions']:
        act, val = line.split(',')
        if int(val) == 1: 
            all_actuators.append(act)

    FSA.loadFile(aut_file, all_sensors, all_actuators, spec_data['SETTINGS']['Customs'])

    ############################
    # Start status/control GUI #
    ############################

    # TODO: implement NO-GUI option

    global runFSA, fd_gui_output, fd_gui_input, guiListenInitialized  # For sharing with thread

    runFSA = False  # Start out paused
    guiListenInitialized = False

    # Create a subprocess
    print "Starting GUI window and listen thread..."
    p_gui = subprocess.Popen(os.path.join(".","simGUI.py"), stderr=subprocess.PIPE, stdin=subprocess.PIPE) # TODO: this will only work if we're in the root working directory
    fd_gui_output = p_gui.stderr
    fd_gui_input = p_gui.stdin

    # Create new thread to communicate with subwindow
    guiListenThread = threading.Thread(target = guiListen)
    guiListenThread.start()

    # Block until the GUI listener gets the go-ahead from the subwindow
    while not guiListenInitialized:
        time.sleep(0.05) # We need to sleep to give up the CPU

    # Tell GUI to load background image
    img_file = os.path.join(project_root, project_basename) + "_simbg.png"
    print >>fd_gui_input, "BG:" + img_file

    # Forward all messages to the GUI window
    sys.stdout = fd_gui_input
    
    #############################
    # Begin automaton execution #
    #############################
    
    last_gui_update_time = 0

    ### Wait for the initial start command

    while not runFSA:
        time.sleep(0.05) # We need to sleep to give up the CPU

    ### Figure out where we should start from

    if 'InitialRegion' in exp_cfg_data: 
        init_region = int(exp_cfg_data['InitialRegion'][0])
    else:
        print "WARNING: initial region auto-detection not yet implement" # TODO: determine initial region
        init_region = 0

    ### Have the FSA find a valid initial state

    # Figure out our initially true outputs
    init_outputs = []
    for prop in exp_cfg_data['InitialTruths']:
        if prop not in all_sensors:
            init_outputs.append(prop)

    init_state = FSA.chooseInitialState(init_region, init_outputs)

    if init_state is None:
        print "Unable to execute. Quitting..."
        sys.exit(0)
    else:
        print "Starting from state %s." % init_state.name

    ### Get everything moving

    while True:

        # Idle if we're not running
        while not runFSA:
            drive_handler.setVelocity(0,0)
            time.sleep(0.05) # We need to sleep to give up the CPU

        tic = time.time() 

        FSA.runIteration()
        
        ### Update GUI
        #print >> fd_gui_input, pose_handler.getPose()
        #for sensor in all_sensors:
        #    print >> fd_gui_input, sensor + " is " + str(sensor_handler.getSensorValue(sensor))

        toc = time.time()

        # TODO: Possibly implement max rate-limiting?
        #while (toc - tic) < 0.05:
        #   time.sleep(0.01)
        #   toc = time.time()

        # Update GUI, no faster than 20Hz
        if time.time() - last_gui_update_time > 0.05:
            print "Running at approximately %.2fHz..." % (1/(toc-tic))
            pose = pose_handler.getLastPose()[0:2]
            print "POSE:%d,%d" % tuple(map(int, rev_coordmap(pose[0:2])))

            last_gui_update_time = time.time()


if __name__ == "__main__":
    main(sys.argv)

