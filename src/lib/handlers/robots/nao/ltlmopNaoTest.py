

"""
Test file for the LTLMoP implementation of Nao
"""

import time
import naoInit, naoActuator, naoSensor
from naoqi import ALProxy

tracker = ALProxy("ALRedBallTracker", 'maeby.local', 9559)
motion = ALProxy("ALMotion", 'maeby.local', 9559)

nInit = naoInit.initHandler(1, ip = 'maeby.local')
shared = nInit.getSharedData()

nSens = naoSensor.naoSensorHandler(1, shared)
nActu = naoActuator.naoActuatorHandler(1, shared)


print ">>>>>>    START    <<<<<<<<<<<"

nSens.seeRedBall(5, True)
nActu.startActiveTracking(True, True)
nActu.startActiveTracking(True, False)

for x in range(50):
    print nSens.seeRedBall(5, False)
    print shared["SENSOR_DATA"]["redBallData"]
    time.sleep(.1)


motion.setStiffnesses("Body", 1)

##nActu.moveHandTo(100,0,0, True, True)
##nActu.moveHandTo(150,-80,150, True, False)

nActu.grabRedBall(35, True, True)
nActu.grabRedBall(35, True, False)

nActu.startActiveTracking(False, False)

time.sleep(2)
#motion.setStiffnesses("Body", 0)

print ">>>>>>    END    <<<<<<<<<<<"

