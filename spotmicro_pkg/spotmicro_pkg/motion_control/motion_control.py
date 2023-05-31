import numpy as np
import time
import math
import threading

from mpu6050 import mpu6050

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup

from degree_msg_interface.msg import JointAngle
from spotmicro_pkg.robot_class.trotting_gait import TrottingGait
from spotmicro_pkg.robot_class.spotmicroai import Robot
from spotmicro_pkg.robot_class.controller import Controllers

class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control')
        self.key_value = None
        self.controller = Controllers()
        self.trotting_gait = TrottingGait()
        self.robot = Robot()
        self.spur_width = self.robot.W/2+20
        self.key_array = ['w', 'a', 's', 'd', 'q', 'e']
        self.lp = np.array([[120, -50, self.spur_width, 1],
                            [120, -50, -self.spur_width, 1],
                            [-60, -55, self.spur_width, 1],
                            [-60, -55, -self.spur_width, 1]])

        self.joint_angle = []
        self.joy_x, self.joy_y, self.joy_z, self.joy_rz = 128, 128, 128, 128
        self.height = 30
        self.rtime = time.time()

        self.callback_group = ReentrantCallbackGroup()
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.keyboard_input_subscriber = self.create_subscription(
            String,
            'keyboard_input',
            self.subscrib_keyboard_input_msg,
            QOS_RKL10V,
            callback_group=self.callback_group
        )

        #TODO self.imu_subscriber

        self.motor_degree_publisher = self.create_publisher(JointAngle, 'motor_degree', QOS_RKL10V)
        self.leg_point_publisher = self.create_publisher(JointAngle, 'leg_point', QOS_RKL10V)

        self.run_thread = threading.Thread(target=self.run)
        self.keyboard_input_thread = threading.Thread(target=self.subscrib_keyboard_input_msg)

        self.run_thread.start()
        self.keyboard_input_thread.start()

    def subscrib_keyboard_input_msg(self, msg):
        self.key_value = msg.data
        self.get_logger().info('keyboard input msg: {0}'.format(msg.data))

    def publish_leg_point(self, leg_point):
        msg = JointAngle()
        leg_point = leg_point.tolist()
        msg.lp1 = leg_point[0]
        msg.lp2 = leg_point[1]
        msg.lp3 = leg_point[2]
        msg.lp4 = leg_point[3]
        self.leg_point_publisher.publish(msg)

    def reset(self):
        self.rtime = time.time()

    def run(self):
        while True:
            d = time.time()-self.rtime

            if self.key_value in self.key_array and self.key_value is not None:
                current_lp = self.trotting_gait.positions(d, self.key_array.index(self.key_value))
                self.robot.feetPosition(current_lp)
            else:
                if self.key_value == 'up':
                    self.lp = np.array([[120, -100, self.spur_width, 1],
                                        [120, -100, -self.spur_width, 1],
                                        [-50, -110, self.spur_width, 1],
                                        [-50, -110, -self.spur_width, 1]])
                elif self.key_value == 'down':
                    self.lp = np.array([[120, -40, self.spur_width, 1],
                                        [120, -40, -self.spur_width, 1],
                                        [-50, -40, self.spur_width, 1],
                                        [-50, -40, -self.spur_width, 1]])
                self.robot.feetPosition(self.lp)

            #TODO imu subscriber값을 bodyRotation에 넣어야 함

            self.robot.bodyRotation((0,math.pi/180*((self.joy_x)-128)/3,-(1/256*self.joy_y-0.5)))
            bodyX=50+0*10
            self.robot.bodyPosition((bodyX, 40+self.height, 0))

            self.publish_leg_point(self.robot.getLp())

            self.joint_angle = self.robot.getAngle()

            if len(self.joint_angle):
                self.controller.servoRotate(self.joint_angle)
                self.get_logger().info('degree: {0}'.format(self.joint_angle))
            self.robot.step()


