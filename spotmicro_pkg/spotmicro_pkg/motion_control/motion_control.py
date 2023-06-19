import numpy as np
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from servo_controller_srv_interface.srv import JointAngle
from spotmicro_pkg.robot_class.trotting_gait import TrottingGait
from spotmicro_pkg.robot_class.spotmicroai import Robot
from spotmicro_pkg.robot_class.imu import MPU

class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control')
        print('motion_control start')
        self.key_value = None
        self.trotting_gait = TrottingGait()
        self.robot = Robot()
        self.mpu = MPU(250, 2, 0.98)
        self.spur_width = self.robot.W/2+20
        self.key_array = ['w', 'a', 's', 'd', 'q', 'e']
        self.lp = np.array([[120, -100, self.spur_width, 1],
                            [120, -100, -self.spur_width, 1],
                            [-60, -100, self.spur_width, 1],
                            [-60, -100, -self.spur_width, 1]])

        self.joint_angle = []
        self.joy_x, self.joy_y, self.joy_z, self.joy_rz = 128, 128, 128, 128
        self.height = 10
        self.rtime = time.time()
        # Set up sensor and calibrate gyro with N points
        self.mpu.setUp()
        self.mpu.calibrateGyro(500)

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

        self.servo_controller_service_client = self.create_client(
            JointAngle,
            'servo_control_operator'
        )
        # self.leg_point_publisher = self.create_publisher(JointAngle, 'leg_point', QOS_RKL10V)

        self.run_thread = threading.Thread(target=self.run)
        self.keyboard_input_thread = threading.Thread(target=self.subscrib_keyboard_input_msg)

        self.run_thread.start()
        self.keyboard_input_thread.start()

    def subscrib_keyboard_input_msg(self, msg):
        self.key_value = msg.data
        self.get_logger().info('keyboard input msg: {0}'.format(msg.data))

    # def publish_leg_point(self, leg_point):
    #     msg = JointAngle()
    #     leg_point = leg_point.tolist()
    #     msg.lp1 = leg_point[0]
    #     msg.lp2 = leg_point[1]
    #     msg.lp3 = leg_point[2]
    #     msg.lp4 = leg_point[3]
    #     self.leg_point_publisher.publish(msg)

    def send_request_servo_controller(self):
        service_request = JointAngle.Request()
        joint_angle = self.joint_angle.tolist()
        service_request.leg1 = joint_angle[0]
        service_request.leg2 = joint_angle[1]
        service_request.leg3 = joint_angle[2]
        service_request.leg4 = joint_angle[3]
        res = self.servo_controller_service_client.call_async(service_request)
        return res

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

            roll, pitch = self.mpu.compFilter()

            self.robot.bodyRotation((roll*math.pi/180, 0, pitch*math.pi/180))
            # self.robot.bodyRotation((0, 0,-(1/256*self.joy_y-0.5)))
            bodyX=50+0*10
            self.robot.bodyPosition((bodyX, 40+self.height, 0))

            # self.publish_leg_point(self.robot.getLp())

            self.joint_angle = self.robot.getAngle()

            if len(self.joint_angle):
                self.send_request_servo_controller()
                self.get_logger().info('degree: {0}'.format(self.joint_angle))
            self.robot.step()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotionControl()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('keyboard Interrupt')
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
