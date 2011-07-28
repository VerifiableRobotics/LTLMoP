from socket import *
from struct import *
from time import *
from numpy import *
from math import *

def rotate(yaw, pitch, roll):
    R = mat([[cos(yaw)*cos(pitch), (cos(yaw)*sin(pitch)*sin(roll))-(sin(yaw)*cos(roll)), (cos(yaw)*sin(pitch)*cos(roll))+(sin(yaw)*sin(roll))],
            [sin(yaw)*cos(pitch), (sin(yaw)*sin(pitch)*sin(roll))+(cos(yaw)*cos(roll)), (sin(yaw)*sin(pitch)*cos(roll))-(cos(yaw)*sin(roll))],
            [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    return R

if (__name__ == '__main__'):
    UDPsock = socket(AF_INET, SOCK_DGRAM)
    UDPsock.bind(("0.0.0.0", 11111))

    while 1:
        bytedata, addr = UDPsock.recvfrom(48)
        data = unpack('dddddd', bytedata)
        print data

        # Rotate the Vicon subject's forward vector by the rotation matrix.
        fwdvec = array([[0],[-1],[0]])               # Dependent on Vicon subject. 
        R = rotate(data[3],data[4],data[5])
        vec = R*fwdvec

        # Find the angle this vector makes with the x-y plane.
        angle = atan2(vec[1],vec[0])
        print angle*(180.0/pi)

        #sleep(1)
