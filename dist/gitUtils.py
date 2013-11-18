import subprocess
import time
import sys
import os

def ensureGitBash(script_path):
    # If on Windows, use Git Bash for the shell
    if sys.platform in ['win32', 'cygwin']:
        # Check if we have access to bash
        cmd = subprocess.Popen(["bash", "--version"],stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=True)

        # Wait for subprocess to finish
        while cmd.returncode is None:
            cmd.poll()
            time.sleep(0.01)

        if cmd.returncode != 0:
            print "Trying to use Git Bash..."

            bash_path = None
            for ev in ["ProgramFiles", "ProgramFiles(x86)", "ProgramW6432"]:
                if ev not in os.environ: continue

                bp = os.path.join(os.environ[ev], 'Git', 'bin', 'bash.exe')

                if os.path.exists(bp):
                    bash_path = bp
                    break

            if bash_path is None:
                print "Couldn't find Git Bash.  Please install Git for Windows."
                print "(See http://code.google.com/p/msysgit/)"
                print
                print "Press [Enter] to quit..."
                raw_input()
                sys.exit(1)

            print "Found Git Bash at %s" % bash_path

            cmd = subprocess.Popen([bash_path, "--login", "-i", "-c", '"%s" "%s"' % (sys.executable, os.path.abspath(script_path))])

            # Wait for subprocess to finish
            try:
                while cmd.returncode is None:
                    cmd.poll()
                    time.sleep(0.01)
            except KeyboardInterrupt:
                cmd.kill()

            sys.exit(0)

if __name__ == "__main__":
	ensureGitBash(__file__)
	cmd = subprocess.Popen(["git", "status"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False)
	print cmd.communicate()[0]
	raw_input("Press any key to quit...")
