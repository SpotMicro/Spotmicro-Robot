import time
import math
import numpy as np
from spotmicro_pkg.robot_class.kinematics import Kinematic
from enum import Enum

class RobotState(Enum):
    OFF = 0     # don't do anything
    READY = 1   # compact, legs together, waiting
    STAND = 2   # standing, feet to the ground
    TROTTING_GAIT=3 # legs away moving up/down 0/3,1/2 / 2 Step
    CRAWL = 4   # 4 Stepped, 1,2,3,0
    CRAWL2 = 5  #4 Stepped, Back first, 2,1,3,0

class Robot:

    def __init__(self, resetFunc=None):

        self.resetFunc=resetFunc
        self.useRealTime = True
        self.fixedTimeStep = 1. / 550

        self.state=RobotState.OFF


        self.angles = []

        self.rot=(0,0,0)
        self.pos=(0,0,0)
        self.t=0

        self.W=75+5+40

        self.Lp = np.array([[120, -100, self.W/2, 1], [120, -100, -self.W/2, 1],
        [-50, -105, self.W/2, 1], [-50, -105, -self.W/2, 1]])

        self.kin = Kinematic()
        self.ref_time = time.time()

    def bodyRotation(self,rot):
        self.rot=rot

    def bodyPosition(self,pos):
        self.pos=pos

    def feetPosition(self,Lp):
        self.Lp=Lp

    def getPos(self):
        pass

    def getAngle(self):
        return self.angles

    def getIMU(self):
        pass

    def step(self):

        if (self.useRealTime):
            self.t = time.time() - self.ref_time
        else:
            self.t = self.t + self.fixedTimeStep

        self.angles = self.kin.calcIK(self.Lp, self.rot, self.pos)

        if (self.useRealTime == False):
            time.sleep(self.fixedTimeStep)

    def getLp(self):
        return self.Lp
