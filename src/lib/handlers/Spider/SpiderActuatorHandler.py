#!/usr/bin/env python
"""
=========================================
dummyActuator.py - Dummy Actuator Handler
=========================================

The actions for the spider. Will be able to change the gaits that the spider will do.
"""

import lib.handlers.handlerTemplates as handlerTemplates

class SpiderActuatorHandler(handlerTemplates.ActuatorHandler):

    def __init__(self, executor, shared_data):
        self.currentGaits = "normal"
        self.loadGait("normal", 1)

        try:
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
            self.gaits = shared_data["Gaits"]
        except NameError:
            print "(ACTU) ERROR: Loco or Gaits not found."
            exit(-1)


    def sitDown(self, actuatorVal, initial=False):
        """
        Will tell the spider to sit down and turn off servos
        """
        if initial:
            return

        if int(actuatorVal) == 0:
            self.standUp('1', False)

        elif self.loco.state != "sitting":
            self.loco.setMoving(False)
            self.loco.streamGait(self.gaits["stand"])
            self.loco.performGait("sitDown")
            self.loco.turnOffServos()
            self.loco.state = "sitting"
            self.loco.currentDir = None

    def standUp(self, actuatorVal, initial=False):
        """
        Will tell the spider to stand up
        """
        if initial or int(actuatorVal) == 0:
            return

        if self.loco.state != "standing":
            self.loco.performGait("standUp")
            self.loco.state = "standing"
            self.loco.currentDir = None

    def wave(self, actuatorVal, initial=False):
        """
        Tell the spider to wave and then return to a standing position
        """
        if initial or int(actuatorVal) == 0:
            return

        self.loco.setMoving(False)
        self.loco.streamGait(self.gaits["stand"])
        self.loco.streamGait(self.gaits["wave"])
        self.loco.streamGait(self.gaits["stand"])
        self.loco.state = "standing"
        self.loco.currentDir = None

    def clap(self, actuatorVal, initial=False):
        """
        Tell the spider to clap and then return to a standing position
        """
        if initial or int(actuatorVal) == 0:
            return

        self.loco.setMoving(False)
        self.loco.streamGait(self.gaits["stand"])
        self.loco.streamGait(self.gaits["clap"])
        self.loco.streamGait(self.gaits["stand"])
        self.loco.state = "standing"
        self.loco.currentDir = None

    def threaten(self, actuatorVal, initial=False):
        """
        Tell the spider to threaten and then return to a standing position
        """
        if initial or int(actuatorVal) == 0:
            return

        dir = self.loco.currentDir

        self.loco.setMoving(False)
        self.loco.streamGait(self.gaits["stand"])
        if dir is None:
            print "(ACTU) no direction"
            self.loco.streamGait(self.gaits["threaten"])
        else:
            print "(ACTU) got dirrection"
            self.loco.streamGait(self.gaits["threaten"].cloneAndRotate(dir))
        self.loco.streamGait(self.gaits["stand"])
        self.loco.state = "standing"
        self.loco.currentDir = None

    def loadGait(self, typeOfGait, actuatorVal, initial=False):
        """
        Load different types of gaits for the spider to use while walking

        typeOfGait (string): The type of gait (ex. normal, incline, gravel)
        """
        if initial:
            return

        if (typeOfGait == "normal" or int(actuatorVal) == 0) and self.currentGaits != "normal":
            self.loco.storeGait(self.gaits["walk1"], 2)
            self.loco.storeGait(self.gaits["walk2"], 4)
            self.currentGaits = "normal"

        elif typeOfGait == "incline" and self.currentGaits != "incline":
            self.loco.storeGait(self.gaits["walk1Incline"], 2)
            self.loco.storeGait(self.gaits["walk2Incline"], 4)
            self.currentGaits = "incline"

        elif typeOfGait == "gravel" and self.currentGaits != "gravel":
            self.loco.storeGait(self.gaits["walk1Gravel"], 2)
            self.loco.storeGait(self.gaits["walk2Gravel"], 4)
            self.currentGaits = "gravel"

        else:
            print "(ACTU) Unknown gait requested"



