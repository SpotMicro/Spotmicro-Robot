from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from degree_msg_interface.msg import JointAngle
from spotmicro_pkg.robot_class.kinematics import Kinematic

import numpy as np
from math import *
import threading

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

import tkinter as tk
from tkinter import ttk

def setupView(limit):
        ax = plt.axes(projection="3d")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Z")
        ax.set_zlabel("Y")
        return ax

class MotionSimulation(Node):
    def __init__(self):
        super().__init__('motion_simulation')
        self.l1=50
        self.l2=20
        self.l3=100
        self.l4=100

        self.L = 140
        self.W = 75

        self.LEG_FRONT = 0
        self.LEG_BACK = 2
        self.LEG_LEFT = 0
        self.LEG_RIGHT =1
        fig = plt.figure()

        self.kinematic = Kinematic()
        self.callback_group = ReentrantCallbackGroup()
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.leg_point_subscriber = self.create_subscription(
            JointAngle,
            'leg_point',
            self.subscrib_leg_point,
            QOS_RKL10V,
            callback_group=self.callback_group
        )
        self.lp = np.array([[120, -100, 80, 1],
                            [120, -100, -80, 1],
                            [-50, -100, 80, 1],
                            [-50, -100, -80, 1]])

        self.run_thread = threading.Thread(target=self.run)
        self.leg_point_subscriber_thread = threading.Thread(target=self.subscrib_leg_point)

        self.run_thread.start()
        self.leg_point_subscriber_thread.start()

    def subscrib_leg_point(self, msg):
        self.lp = []
        self.lp.append(msg.lp1)
        self.lp.append(msg.lp2)
        self.lp.append(msg.lp3)
        self.lp.append(msg.lp4)
        self.get_logger().info('lp: {0} {1} {2} {3}'.format(msg.lp1, msg.lp2, msg.lp3, msg.lp4))

    def drawLegPoints(self,p):
        # draw basic position
        plt.plot([x[0] for x in p],[x[2] for x in p],[x[1] for x in p], 'k-', lw=3)

        #draw body points
        plt.plot([p[0][0]],[p[0][2]],[p[0][1]],'bo',lw=2)

        # draw end points
        plt.plot([p[4][0]],[p[4][2]],[p[4][1]],'ro',lw=2)

    def drawLegPair(self,Tl,Tr,Ll,Lr, LEG_FR):
        Ix=np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.drawLegPoints([Tl.dot(x) for x in self.kinematic.calcLegPoints(self.kinematic.legIK(np.linalg.inv(Tl).dot(Ll)))])
        self.drawLegPoints([Tr.dot(Ix.dot(x)) for x in self.kinematic.calcLegPoints(self.kinematic.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))))])

        # print(self.legIK(np.linalg.inv(Tl).dot(Ll)))
        self.kinematic.thetas[LEG_FR+self.LEG_LEFT]=np.array(self.kinematic.legIK(np.linalg.inv(Tl).dot(Ll)))

        # print(self.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))))
        self.kinematic.thetas[LEG_FR+self.LEG_RIGHT]=np.array(self.kinematic.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))))

    def drawRobot(self,Lp,angles,center):
        (omega,phi,psi)=angles
        (xm,ym,zm)=center

        FP=[0,0,0,1]
        (Tlf,Trf,Tlb,Trb)= self.kinematic.bodyIK(omega,phi,psi,xm,ym,zm)
        CP=[x.dot(FP) for x in [Tlf,Trf,Tlb,Trb]]

        CPs=[CP[x] for x in [0,1,3,2,0]]

        # draw body points with body edges
        plt.plot([x[0] for x in CPs],[x[2] for x in CPs],[x[1] for x in CPs], 'bo-', lw=2)

        self.drawLegPair(Tlf,Trf,Lp[0],Lp[1], self.LEG_FRONT)
        self.drawLegPair(Tlb,Trb,Lp[2],Lp[3], self.LEG_BACK)

    def drawLegPoints(self,p):
        # draw basic position
        plt.plot([x[0] for x in p],[x[2] for x in p],[x[1] for x in p], 'k-', lw=3)

        #draw body points
        plt.plot([p[0][0]],[p[0][2]],[p[0][1]],'bo',lw=2)

        # draw end points
        plt.plot([p[4][0]],[p[4][2]],[p[4][1]],'ro',lw=2)

    def drawLegPair(self,Tl,Tr,Ll,Lr, LEG_FR):
        Ix=np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.drawLegPoints([Tl.dot(x) for x in self.kinematic.calcLegPoints(self.kinematic.legIK(np.linalg.inv(Tl).dot(Ll)))])
        self.drawLegPoints([Tr.dot(Ix.dot(x)) for x in self.kinematic.calcLegPoints(self.kinematic.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))))])

        # print(self.legIK(np.linalg.inv(Tl).dot(Ll)))
        self.kinematic.thetas[LEG_FR+self.LEG_LEFT]=np.array(self.kinematic.legIK(np.linalg.inv(Tl).dot(Ll)))

        # print(self.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))))
        self.kinematic.thetas[LEG_FR+self.LEG_RIGHT]=np.array(self.kinematic.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))))

    def drawRobot(self,Lp,angles,center):
        (omega,phi,psi)=angles
        (xm,ym,zm)=center

        FP=[0,0,0,1]
        (Tlf,Trf,Tlb,Trb)= self.kinematic.bodyIK(omega,phi,psi,xm,ym,zm)
        CP=[x.dot(FP) for x in [Tlf,Trf,Tlb,Trb]]

        CPs=[CP[x] for x in [0,1,3,2,0]]

        # draw body points with body edges
        plt.plot([x[0] for x in CPs],[x[2] for x in CPs],[x[1] for x in CPs], 'bo-', lw=2)

        self.drawLegPair(Tlf,Trf,Lp[0],Lp[1], self.LEG_FRONT)
        self.drawLegPair(Tlb,Trb,Lp[2],Lp[3], self.LEG_BACK)

    def showGraph(self):
        plt.show(block=False)
    def run(self):
        root = tk.Tk()

        while True:
            print('----------')
            setupView(200).view_init(elev=12., azim=28)
            self.drawRobot(self.lp,(0,0,0),(0,0,0))

            graph_thread = threading.Thread(target=self.showGraph)
            graph_thread.start()

            root.update()
            root.mainloop()
