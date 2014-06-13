import serial
import time
import sys
import os

"""
    Used for debugging, add comport as command line argument
    eg. python testServo.py COM6
    
    Sends out direct servo commands to Johnny 5
"""
class testServo:

    def __init__(self):

        comPort = "/dev/tty.usbserial-A600eIiI"
        try:
            #comPort = sys.argv[1]
            self.johnny5Serial = serial.Serial(port = comPort, baudrate = 115200)
        except:
            print("Couldn't connect to Johnny 5")
            sys.exit(-1)

#        self.johnny5Serial.write('VA VC\r')
#        ans = self.johnny5Serial.read(2)
#        ans = ans.encode('hex')
#        va = ans[0]+ans[1]
#        vc = ans[2]+ans[3]
#
#        print int(va,16)
#        print int(vc,16)
                
        # save .cfg file in folder ConfigFile under Johnny5 folder
        self.config = self.initConfig(os.path.join(os.path.dirname(__file__),'ConfigFile','ConfigSSC32.cfg'))
        self.setDefaultPosition()
                
        while 1:
            # read force sensors on both hands
            self.johnny5Serial.write('VA VC\r')
            # read 2 bytes
            sensorData = self.johnny5Serial.read(2)
            sensorData = sensorData.encode('hex')
            # convert sensor readings to integer
            right = sensorData[0]+sensorData[1]
            right = int(right,16)
            left = sensorData[2]+sensorData[3]
            left = int(left,16)
        
            if left>=100:
                print left
            # put right hand forward
#            self.johnny5Serial.write('#3 P2200 T1000\r')
#            self.johnny5Serial.write('#5 P2200 T1000\r')
#            self.johnny5Serial.write('#6 P1200 T1000\r')
#            time.sleep(0.5)
#        
#            self.johnny5Serial.write('#8 P1000 T1000\r')
#            self.johnny5Serial.write('#10 P500 T1000\r')
#            self.johnny5Serial.write('#11 P2000 T1000\r')
#            # open up right hand
#            self.johnny5Serial.write('#12 P1800 T1000\r')
#            time.sleep(0.5)
#            
#            val = 1800
#            self.johnny5Serial.write('VA VC\r')
#            ans = self.johnny5Serial.read(2)
#            ans = ans.encode('hex')
#            va = ans[0]+ans[1]
#            va = int(va,16)
#            while va<100 and val>1300:
#                val = val - 50
#                self.johnny5Serial.write('#12 P%d T1000\r' % val)
#                self.johnny5Serial.write('VA VC\r')
#                ans = self.johnny5Serial.read(2)
#                ans = ans.encode('hex')
#                va = ans[0]+ans[1]
#                va = int(va,16)
#                print va
#                time.sleep(0.5)
#            self.johnny5Serial.write('#12 P1800 T1000\r')
#            time.sleep(0.5)
    
        self.johnny5Serial.close()

    def initConfig(self, cfgFileName):
        """
            Reads in default configuration file and saves.
            
            8 lines of data for each servo, 16 servos for Johnny 5,
            In each data set of a servo, only lines 4-8 are of interests:
            
            4: servo value at neutral position
            5: Min servo value
            6: Max servo value
            7: Min servo degree
            8: Max servo degree
            
            Generate a 2D array "config" in the following format:
            
            index:          0          1         2         3          4
            Servo #0 Neutral_ servo Min_servo Max_servo Min_degree Max_degree
            Servo #1       ...
            .
            .
            .
            """
        cfg = [data.strip('\r\n') for data in open(cfgFileName)]
        # config is a 16x5 array, initialized with all 0
        config = [[0 for i in range(5)] for j in range(16)]
        for servoNum in range(16):
            config[servoNum] = map(int, cfg[8*servoNum+3:8*servoNum+8])
        return config
            
    def setDefaultPosition(self):
        """
        Set Johnny 5 servos to default positions
        """
        time = 1000
        # for servo# 0-15
        for i in range(16):
            self.johnny5Serial.write('#%d P%d T%d\r' % (i, int(self.config[i][0]), time))

if __name__ == '__main__':
    j5=testServo()