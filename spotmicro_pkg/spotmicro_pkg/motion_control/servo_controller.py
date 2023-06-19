import numpy as np
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio
import time

import rclpy
from rclpy.node import Node
from servo_controller_srv_interface.srv import JointAngle

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self._i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        self._pca_1 = PCA9685(self._i2c_bus0, address=0x41)
        self._pca_1.frequency = 60
        self._pca_2 = PCA9685(self._i2c_bus0, address=0x40)
        self._pca_2.frequency = 60

        self._servos = list()

        for i in range(0, 16):
            if i < 3 or i > 12:
                self._servos.append(servo.Servo(self._pca_1.channels[i], min_pulse=500, max_pulse=2500))
        for i in range(0, 16):
            if i < 3 or i > 12:
                self._servos.append(servo.Servo(self._pca_2.channels[i], min_pulse=500, max_pulse=2500))
        print('ready servo controller')
        self._servo_offsets = [180, 98, 90, 11, 90, 90, 187, 92, 80, -1, 93, 90]

        self._val_list = [ x for x in range(12) ]

        self._thetas = []
        self.joint_angle = []
        self._history_val_list = [ x for x in range(12) ]

        self.arithmetic_service_server = self.create_service(
            JointAngle,
            'servo_control_operator',
            self.get_servo_degree_operator,
        )
    def get_servo_degree_operator(self, request, response):
        self.joint_angle = []
        self.joint_angle.append(request.leg1)
        self.joint_angle.append(request.leg2)
        self.joint_angle.append(request.leg3)
        self.joint_angle.append(request.leg4)
        joint_angle = np.array(self.joint_angle, dtype='float32')
        response.confirm = self.servoRotate(joint_angle)
        # self.get_logger().info('response: {0}'.format(response.confirm))
        # self.get_logger().info('jointAngle: {0} {1} {2} {3}'.format(self.joint_angle[0], self.joint_angle[1], self.joint_angle[2], self.joint_angle[3]))
        return response

    def getDegreeAngles(self, La):
        La *= 180/np.pi
        La = [ [ int(x) for x in y ] for y in La ]

        self._thetas = La

    def angleToServo(self, La):
        self.getDegreeAngles(La)

        #FL Lower
        self._val_list[0] = self._servo_offsets[0] - self._thetas[0][2]
        #FL Upper
        self._val_list[1] = self._servo_offsets[1] - self._thetas[0][1]
        #FL Shoulder
        self._val_list[2] = self._servo_offsets[2] + self._thetas[0][0]

        #FR Lower
        self._val_list[3] = self._servo_offsets[3] + self._thetas[1][2]
        #FR Upper
        self._val_list[4] = self._servo_offsets[4] + self._thetas[1][1]
        #FR Shoulder
        self._val_list[5] = self._servo_offsets[5] - self._thetas[1][0]

        #BL Lower
        self._val_list[6] = self._servo_offsets[6] - self._thetas[2][2]
        #BL Upper
        self._val_list[7] = self._servo_offsets[7] - self._thetas[2][1]
        #BL Shoulder, Formula flipped from the front
        self._val_list[8] = self._servo_offsets[8] - self._thetas[2][0]

        #BR Lower.
        self._val_list[9] = self._servo_offsets[9] + self._thetas[3][2]
        #BR Upper
        self._val_list[10] = self._servo_offsets[10] + self._thetas[3][1]
        #BR Shoulder, Formula flenlipped from the front
        self._val_list[11] = self._servo_offsets[11] + self._thetas[3][0]

    def getServoAngles(self):
        return self._val_list

    def servoRotate(self, thetas):
        self.angleToServo(thetas)

        for x in range(len(self._val_list)):

            if (self._val_list[x] > 180):
                print("Over 180!!")
                self._val_list[x] = 179
            if (self._val_list[x] <= 0):
                print("Under 0!!")
                self._val_list[x] = 1
            self._servos[x].angle = float(self._val_list[x])
        for i in range(len(self._val_list)):
            self._history_val_list[i] = self._val_list[i]
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
