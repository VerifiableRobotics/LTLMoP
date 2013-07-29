#!/usr/bin/env python

#########################################################################
# This is a script to automatically run commands necessary for initial
# preparation of a LTLMoP installation
#########################################################################

import os, sys, time
import subprocess
import compileall

# TODO: use distutils?

def runProgramWithLiveOutput(wd, program, *args):
    if os.name == "nt":
        err_types = (OSError, WindowsError)
    else:
        err_types = OSError

    try:
        p = subprocess.Popen([program] + list(args), cwd=wd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except err_types as (errno, strerror):
        print "ERROR: " + strerror
        return

    while p.returncode is None:
        output = p.stdout.readline() # Blocking :(
        print output,

        # Check the status of the process
        p.poll()

# Check python version
if sys.hexversion < 0x02070000:
    print "ERROR: LTLMoP requires Python 2.7+"
    print "Press any key to quit..."
    raw_input()
    sys.exit(2)

# Find src/ directory
src_dir = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "src")

print ">>> Pre-compiling all modules..."
os.chdir(src_dir)
compileall.compile_dir("lib", quiet=True)

print ">>> Building synthesis subsystem..."
jtlv_dir = os.path.join(src_dir, "etc", "jtlv")
runProgramWithLiveOutput(jtlv_dir, os.path.join(jtlv_dir, "build.sh"))

print ">>> Setting up SLURP parsing system..."
print "Note: This step involves downloading a large file, and is optional."
skip_slurp = None
while skip_slurp is None:
    skip_slurp_response = raw_input("Skip this step [y/N]? ")
    if skip_slurp_response.lower() == "y":
        print "Skipping."
        skip_slurp = True
    elif skip_slurp_response.lower() == "n" or skip_slurp_response == "":
        skip_slurp = False

if not skip_slurp:
    slurp_dir = os.path.join(src_dir, "etc", "SLURP")
    runProgramWithLiveOutput(slurp_dir, "python", "-u", os.path.join(slurp_dir, "download.py"))

print ">>> Done!"

if os.name == "nt":
    raw_input("Press any key to quit...")
