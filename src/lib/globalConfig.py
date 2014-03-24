"""
Module for loading and saving LTLMoP global configuration file.
Currently, this only sets up logging.
"""

import logging
import ConfigParser
import sys, os
import time

def get_ltlmop_root():
    # Climb the tree to find out where we are
    p = os.path.abspath(__file__)
    t = ""
    while t != "src":
        (p, t) = os.path.split(p)
        if p == "" or p == "/":
            print "I have no idea where I am; this is ridiculous"
            return None

    return os.path.join(p, "src")

def setupLogging(loggerLevel=None):
    # Set up loggers for printing error messages
    class ColorLogFormatter(logging.Formatter):
        def __init__(self, *args, **kwds):
            super(ColorLogFormatter, self).__init__(*args, **kwds)
            self.plain_formatter = logging.Formatter(" [ %(module)s ] %(message)s")
            self.detailed_formatter = logging.Formatter(" --> [%(levelname)s] (%(pathname)s, line %(lineno)s): %(message)s")

        def colorize(self, level, string):
            if sys.platform in ['win32', 'cygwin']:
                # Message with color is not yet supported in Windows
                return string
#             elif not hasattr(sys.stderr, "isatty") or not sys.stderr.isatty():
#                 # Only try to colorize if outputting to a terminal 
#                 return string
            else:
                colors = {'ERROR': 91, 'WARNING': 93, 'INFO': 97, 'DEBUG': 94}
                return "\033[{0}m{1}\033[0m".format(colors[level], string)

        def format(self, record):
            if record.levelname == "INFO":
                precolor = self.plain_formatter.format(record)
            else:
                precolor = self.detailed_formatter.format(record)

            return self.colorize(record.levelname, precolor)
            
    logger = logging.getLogger()
    h = logging.StreamHandler()
    f = ColorLogFormatter()
    h.setFormatter(f)
    if not logger.handlers:
        logger.addHandler(h)

    cfg = ConfigParser.ConfigParser()

    try:
        cfg.read(os.path.join(get_ltlmop_root(), "global.cfg"))
        loggerLevel = cfg.get("logging", "level").lower()
    except:
        logging.warning("Could not parse global.cfg file; using defaults")
        loggerLevel = "info"


    if loggerLevel == 'error':
        logger.setLevel(logging.ERROR)
    elif loggerLevel == 'warning':
        logger.setLevel(logging.WARNING)
    elif loggerLevel == 'info':
        logger.setLevel(logging.INFO)
    elif loggerLevel == 'debug':
        logger.setLevel(logging.DEBUG)

# Choose the timer func with maximum accuracy for given platform
if sys.platform in ['win32', 'cygwin']:
    best_timer = time.clock
else:
    best_timer = time.time

# Set-up logging automatically on import
setupLogging()
