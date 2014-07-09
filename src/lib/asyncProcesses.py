import os
import logging
import time
import threading
import subprocess

class AsynchronousProcessThread(threading.Thread):
    def __init__(self, cmd, callback, logFunction):
        """
        Run a command asynchronously, calling a callback function (if given) upon completion.
        If a logFunction is given, stdout and stderr will be redirected to it.
        Otherwise, these streams are printed to the console.
        """

        self.cmd = cmd
        self.callback = callback
        self.logFunction = logFunction

        self.running = False

        threading.Thread.__init__(self)

        self.startComplete = threading.Event()

        # Auto-start
        self.daemon = True
        self.start()

    def kill(self):
        print "Killing process `%s`..." % ' '.join(self.cmd)

        self.running = False

        try:
            # This should cause the blocking readline() in the run loop to return with an EOF
            self.process.kill()
        except OSError:
            # TODO: Figure out what's going on when this (rarely) happens
            print "Ran into an error killing the process.  Hopefully we just missed it..."

    def run(self):
        if os.name == "nt":
            err_types = (OSError, WindowsError)
        else:
            err_types = OSError

        # Start the process
        try:
            if self.logFunction is not None:
                self.process = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=False, bufsize=-1)
            else:
                self.process = subprocess.Popen(self.cmd, bufsize=-1)
        except err_types as (errno, strerror):
            print "ERROR: " + strerror
            self.startComplete.set()
            return

        self.running = True
        self.startComplete.set()

        # Sit around while it does its thing
        while self.process.returncode is None:
            # Make sure we aren't being interrupted
            if not self.running:
                return

            # Output to either a RichTextCtrl or the console
            if self.logFunction is not None:
                output = self.process.stdout.readline() # Blocking :(

                # Check for EOF (indicated by an empty string). We really
                # shouldn't have to do this, because we are polling for the
                # returncode once per readline, but for some reason on Windows
                # we will get a couple hundred EOFs before the process is marked
                # as completed.

                if output == '':
                    break

                self.logFunction(output)

            # Check the status of the process
            self.process.poll()

            # If we're not logging, limit our poll frequency
            # (In the case of logging, readline() effectively does something similar)
            if self.logFunction is None:
                time.sleep(0.01)

        # Grab any remaining output (often an error)
        if self.logFunction is not None:
            output = self.process.stdout.readlines()
            for line in output:
                self.logFunction(line)

        # Call any callback function if terminated succesfully
        if self.callback is not None and self.running:
            self.callback()

