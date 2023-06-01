import time
import numpy as np
import math

class TrottingGait:

    def __init__(self):
        self.step_gain = 0.8
        self.maxSl=2
        self.bodyPos=(0,100,0)
        self.bodyRot=(0,0,0)
        self.t0=500
        self.t1=500 #leg lift
        self.Sl=50.0
        self.Sw=0
        self.Sh=5 #100
        self.Sa=0
        self.Spf=87
        self.Spr=77
        self.Fo=100
        self.Ro=70
        self.Sra = (self.Spf**2+((self.Fo+self.Ro)/2)**2)

        self.Rc=[-50,0,0,1] # rotation center

        self.calcLegFucntions = [self.forwardMove, self.leftMove, self.backMove, self.rightMove, self.turnLeftMove, self.turnRightMove]

    """
    calculates the Lp - LegPosition for the configured gait for time t and original Lp of x,y,z
    """
    def calcLeg(self, t, startLp, endLp):
        if(t<self.t0): # drag foot over ground

            tp=1/(self.t0)
            diffLp=endLp-startLp
            curLp=startLp+diffLp*tp
            psi=-((math.pi/180*self.Sa)/2)+(math.pi/180*self.Sa)*tp
            Ry = np.array([[np.cos(psi),0,np.sin(psi),0],
                    [0,1,0,0],
                    [-np.sin(psi),0,np.cos(psi),0],[0,0,0,1]])
            #Tlm = np.array([[0,0,0,-self.Rc[0]],[0,0,0,-self.Rc[1]],[0,0,0,-self.Rc[2]],[0,0,0,0]])
            curLp=Ry.dot(curLp)
            return curLp

        elif(t<self.t0+self.t1): # Lift foot
            td= t-self.t0
            tp=1/(self.t1/td)

            diffLp=startLp-endLp
            curLp=endLp+diffLp*tp
            curLp[1]+=self.Sh*math.sin(math.pi*tp)
            return curLp

    def backMove(self,t,x,y,z):
        startLp=np.array([x-self.Sl/2.0,y,z-self.Sw,1])
        endY=0 #-0.8 # delta y to jump a bit before lifting legs
        endLp=np.array([x+self.Sl/2,y+endY,z+self.Sw,1])

        return self.calcLeg(t,startLp,endLp)

    def forwardMove(self,t,x,y,z):
        startLp=np.array([x+self.Sl,y,z-self.Sw,1])
        endY=0 #-0.8 # delta y to jump a bit before lifting legs
        endLp=np.array([x,y+endY,z+self.Sw,1])

        return self.calcLeg(t,startLp,endLp)

    def leftMove(self,t,x,y,z):
        startLp=np.array([x,y,z-self.Sw,1])

        endY=0 #-0.8 # delta y to jump a bit before lifting legs
        endLp=np.array([x,y+endY,z+self.Sw+self.Sl/2,1])

        return self.calcLeg(t,startLp,endLp)

    def rightMove(self,t,x,y,z):
        startLp=np.array([x,y,z-self.Sw,1])
        endY=0 #-0.8 # delta y to jump a bit before lifting legs
        endLp=np.array([x,y+endY,z+self.Sw-self.Sl/2,1])

        return self.calcLeg(t,startLp,endLp)

    def turnLeftMove(self,t,x,y,z):
        startLp=np.array([x,y,z-self.Sw,1])
        endY=0
        if x > 0:

            endLp=np.array([math.sqrt(self.Sra-(z-self.Sl/8)**2), y+endY, z-self.Sl/8, 1])
        else:
            endLp=np.array([math.sqrt(self.Sra-(z+self.Sl/8)**2), y+endY, z+self.Sl/8, 1])

        return self.calcLeg(t,startLp,endLp)

    def turnRightMove(self,t,x,y,z):
        startLp=np.array([x,y,z-self.Sw,1])
        endY=0
        if x < 0:
            endLp=np.array([math.sqrt(self.Sra-(z-self.Sl/8)**2), y+endY, z-self.Sl/8, 1])
        else:
            endLp=np.array([math.sqrt(self.Sra-(z+self.Sl/8)**2), y+endY, z+self.Sl/8, 1])

        return self.calcLeg(t,startLp,endLp)

    def stepLength(self,len):
        self.Sl=len

    def positions(self,t,key_idx):
        spf=self.Spf
        spr=self.Spr
        # self.Sh=60.0
        Tt=(self.t0+self.t1)
        Tt2 = Tt/2
        rd = Tt2/2
        # rd=0 # rear delta - unused - maybe stupid
        td=(t*1000)%Tt
        t2=(t*1000-Tt2)%Tt
        rtd=(t*1000-rd)%Tt # rear time delta
        rt2=(t*1000-Tt2-rd)%Tt
        #TODO Rear 3/4 delayed
        # Tt4=Tt/4
        # td=(t*1000-Tt4-Tt4*3)%Tt
        # t2=(t*1000-(Tt4*(2/4))-Tt4*3)%Tt
        # rtd=(t*1000-(Tt4*(1/4))-Tt4*3)%Tt
        # rt2=(t*1000-(Tt4*(3/4))-Tt4*3)%Tt
        Fx=self.Fo
        Rx=-1*self.Ro
        Fy=-100
        Ry=-100
        r=np.array([self.calcLegFucntions[key_idx](td,Fx,Fy,spf),self.calcLegFucntions[key_idx](t2,Fx,Fy,-spf),self.calcLegFucntions[key_idx](rt2,Rx,Ry,spr),self.calcLegFucntions[key_idx](rtd,Rx,Ry,-spr)])
        #print(r)
        return r

