"""
================================================================================
HexapodInitHandler.py - The Hexapod's Init Handler
================================================================================
"""

import serial

import lib.handlers.handlerTemplates as handlerTemplates

class HexapodInitHandler(handlerTemplates.InitHandler):

    
    def __init__(self, executor, comPort):
        """
        The init handler of Hexapod

        comPort (string): The comport to connect to (default="COM5")
        """

        self.hexapodSer = None   #serial port for hexapod
        #Defaults
        self.baud = 9600     #set baud rate
        self.timeout = 0.5     #in seconds

        try:
            self.hexapodSer = serial.Serial(port = comPort, baudrate =
                                           self.baud, timeout = self.timeout)
                                           
            # set robot to neutral position
            self.hexapodSer.write("b")
            self.hexapodSer.flush()
            # Wait for "setup complete" (q)
            init_response = self.hexapodSer.read()

            while init_response == '':
                init_response = self.hexapodSer.read()
        except:
            print ("(INIT) ERROR: Couldn't connect to hexapod")
            exit(-1)

    def _stop(self):
        print "(INIT) Shutting down serial port!"
        self.hexapodSer.close()
       
    def getSharedData(self):
        # TODO: Return a dictionary of any objects that will need to be shared with
        # other handlers
        return {"hexapodSer":self.hexapodSer}

if __name__ == "__main__":
    h = HexapodInitHandler(None,'COM8')