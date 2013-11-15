#!/usr/bin/env python

#########################################################################
# This is a script to automatically run commands necessary for initial
# preparation of a LTLMoP installation
#########################################################################

import os, sys, time
import subprocess
import compileall
import catUtils
import urllib
import zipfile
import shutil
import re, glob
import gitUtils

# TODO: use distutils?

def runProgramWithLiveOutput(wd, program, args=None, shell=False):
    if os.name == "nt":
        err_types = (OSError, WindowsError)
    else:
        err_types = OSError

    if args is None:
        args = []

    print "-> Running command '{} {}'...".format(program, " ".join(args))
    try:
        p = subprocess.Popen([program] + args, cwd=wd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=shell)
    except err_types as (errno, strerror):
        print "ERROR: " + strerror
        return

    while p.returncode is None:
        output = p.stdout.readline() # Blocking :(
        print output.strip()

        # Check the status of the process
        p.poll()

if __name__ == "__main__":
    # Make sure we are not running on a network drive on Windows
    if os.path.abspath(__file__).startswith(r"\\"):
        print "ERROR: Sorry, this script cannot be run from a network drive."
        #             ... (because windows UNC paths seem to break things)
        print "Please move the LTLMoP folder to a local disk and run this script again."
        print "(You can then move it back later.)" 
        print
        print "Press any key to quit..."
        raw_input()
        sys.exit(2)

    gitUtils.ensureGitBash(__file__)

    # Check python version
    if sys.hexversion < 0x02070000:
        print "ERROR: LTLMoP requires Python 2.7+"
        print "Press any key to quit..."
        raw_input()
        sys.exit(2)

    catUtils.patrickSays("Let's get your LTLMoP installation ready for use.")

    # Find src/ directory
    root_dir = os.path.normpath(os.path.join(os.path.abspath(os.path.dirname(__file__)), ".."))
    src_dir = os.path.join(root_dir, "src")

    print "\n>>> Pre-compiling all modules...\n"
    os.chdir(src_dir)
    compileall.compile_dir("lib", quiet=True)

    print "\n>>> Building synthesis subsystem...\n"
    jtlv_dir = os.path.join(src_dir, "etc", "jtlv")
    runProgramWithLiveOutput(jtlv_dir, os.path.join(jtlv_dir, "build.sh"), shell=True)

    print "\n>>> Checkout out submodules...\n"
    slurp_dir = os.path.join(src_dir, "etc", "SLURP")

    isgitrepo = os.path.exists(os.path.join(root_dir, ".git"))
    if isgitrepo:
        runProgramWithLiveOutput(root_dir, "git", "submodule init".split(), shell=False)
        runProgramWithLiveOutput(root_dir, "git", "submodule update".split(), shell=False)
    else:
        # Manually download submodules zipballs (warning: this method cannot track actual submodule references,
        # so will always fetch the tip of master

        print "-> Downloading archive..."
        urllib.urlretrieve("https://github.com/PennNLP/SLURP/archive/master.zip", os.path.join(slurp_dir, "master.zip"))

        print "-> Unzipping archive..."
        with zipfile.ZipFile(os.path.join(slurp_dir, "master.zip")) as slurp_zip:
            slurp_zip.extractall(slurp_dir)

        # The zipfile contains an extra directory level, which we need to get rid of:
        for fn in glob.iglob(os.path.join(slurp_dir, "SLURP-master", "*")):
            try:
                shutil.move(fn, slurp_dir)
            except shutil.Error as e:
                print "Warning: {}".format(e)

        shutil.rmtree(os.path.join(slurp_dir, "SLURP-master"), ignore_errors=True)
        os.remove(os.path.join(slurp_dir, "master.zip"))
     
    print "\n>>> Setting up SLURP parsing system...\n"
    print "Note: This step involves downloading a large file, and is optional."
    skip_slurp = None
    while skip_slurp is None:
        skip_slurp_response = raw_input("Skip this step [y/N]? ")
        if skip_slurp_response.lower() == "y":
            print "-> Skipping."
            skip_slurp = True
        elif skip_slurp_response.lower() == "n" or skip_slurp_response == "":
            skip_slurp = False

    if not skip_slurp:
        runProgramWithLiveOutput(slurp_dir, sys.executable, ["-u", os.path.join(slurp_dir, "download.py")], shell=False)

    print "\n>>> Done!\n"

    catUtils.patrickSays("Alright. Enjoy!")
    print

    if os.name == "nt":
        raw_input("Press any key to quit...")
