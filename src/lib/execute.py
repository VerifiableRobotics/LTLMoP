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
import fileMethods, regions, fsa, project

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
            print "PAUSED."
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

    proj = project.Project()
    proj.loadProject(spec_file)
    proj.rfiold = proj.rfi
    proj.rfi = proj.loadRegionFile(decomposed=True)

    ##########################
    # Initialize each module #
    ##########################

    proj.lookupHandlers()

    # Import the relevant handlers
    print "Importing handler functions..."
    
    proj.runInitialization()
    proj.importHandlers()

    #######################
    # Load automaton file #
    #######################
    
    print "Loading automaton..."

    FSA = fsa.Automaton(proj.rfi.regions, proj.sensor_handler, proj.actuator_handler, proj.motion_handler)

    FSA.loadFile(aut_file, proj.all_sensors, proj.all_actuators, proj.spec_data['SETTINGS']['Customs'])

    ############################
    # Start status/control GUI #
    ############################

    if show_gui:
        global runFSA, fd_gui_output, fd_gui_input, guiListenInitialized  # For sharing with thread

        runFSA = False  # Start out paused
        guiListenInitialized = False

        # Create a subprocess
        print "Starting GUI window and listen thread..."
        p_gui = subprocess.Popen(["python",os.path.join(proj.ltlmop_root, "lib", "simGUI.py")], stderr=subprocess.PIPE, stdin=subprocess.PIPE)

        fd_gui_output = p_gui.stderr
        fd_gui_input = p_gui.stdin

        # Create new thread to communicate with subwindow
        guiListenThread = threading.Thread(target = guiListen)
        guiListenThread.start()

        # Block until the GUI listener gets the go-ahead from the subwindow
        while not guiListenInitialized:
            time.sleep(0.05) # We need to sleep to give up the CPU

        # Tell GUI to load background image
        print >>fd_gui_input, "BG:" + proj.getFilenamePrefix() + ".spec"

        # Forward all messages from here on to the GUI window
        sys.stdout = fd_gui_input
    
    #############################
    # Begin automaton execution #
    #############################
    
    last_gui_update_time = 0

    ### Wait for the initial start command

    if show_gui:
        while not runFSA:
            time.sleep(0.05) # We need to sleep to give up the CPU
    else:
        raw_input('Press enter to begin...')
        runFSA = True

    ### Figure out where we should start from

    if 'InitialRegion' in proj.exp_cfg_data: 
        init_region = int(proj.exp_cfg_data['InitialRegion'][0])
        init_region = proj.rfi.indexOfRegionWithName(proj.regionMapping[proj.rfiold.regions[init_region].name][0])
    else:
        print "WARNING: Initial region auto-detection not yet implemented" # TODO: determine initial region
        init_region = 0

    print proj.rfi.regions[init_region].name
    
    ### Have the FSA find a valid initial state

    # Figure out our initially true outputs
    init_outputs = []
    for prop in proj.exp_cfg_data['InitialTruths']:
        if prop not in proj.all_sensors:
            init_outputs.append(prop)

    init_state = FSA.chooseInitialState(init_region, init_outputs)

    if init_state is None:
        print "No suitable initial state found; unable to execute. Quitting..."
        sys.exit(-1)
    else:
        print "Starting from state %s." % init_state.name

    ### Get everything moving

    avg_freq = 0

    while True:
        # Idle if we're not running
        while not runFSA:
            proj.drive_handler.setVelocity(0,0) 
            time.sleep(0.05) # We need to sleep to give up the CPU

        tic = time.time() 

        FSA.runIteration()
        
        toc = time.time()

        # TODO: Possibly implement max rate-limiting?
        #while (toc - tic) < 0.05:
        #   time.sleep(0.01)
        #   toc = time.time()

        # Update GUI, no faster than 20Hz
        if time.time() - last_gui_update_time > 0.05:
            avg_freq = 0.9*avg_freq + 0.1*1/(toc-tic) # IIR filter
            print "Running at approximately %dHz..." % int(avg_freq)
            pose = proj.pose_handler.getPose(cached=True)[0:2]
            print "POSE:%d,%d" % tuple(map(int, proj.coordmap_lab2map(pose)))

            last_gui_update_time = time.time()


if __name__ == "__main__":
    main(sys.argv)

