import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from sensor_msgs.msg import Imu

import smbus            #import SMBus module of I2C

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

class ImuPublisher(Node):
    def __init__(self):
        super().__init__()
        self.MPU_Init()
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.imu_publisher = self.create_publisher(Imu, '/Imu', QOS_RKL10V)
        self.timer = self.create_timer(0.1, self.publish_imu_msg)

    def MPU_Init():
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)

        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

    def read_raw_data(addr):
        #Accelero and Gyro value are 16-bit
            high = bus.read_byte_data(Device_Address, addr)
            low = bus.read_byte_data(Device_Address, addr+1)

            #concatenate higher and lower value
            value = ((high << 8) | low)

            #to get signed value from mpu6050
            if(value > 32768):
                    value = value - 65536
            return value


    def publish_imu_msg(self):
        msg = Imu()

        msg.linear_acceleration.x = self.read_raw_data(ACCEL_XOUT_H) / 16384.0
        msg.linear_acceleration.y = self.read_raw_data(ACCEL_YOUT_H) / 16384.0
        msg.linear_acceleration.z = self.read_raw_data(ACCEL_ZOUT_H) / 16384.0

        msg.angular_velocity.x = self.read_raw_data(GYRO_XOUT_H) / 131.0
        msg.angular_velocity.y = self.read_raw_data(GYRO_YOUT_H) / 131.0
        msg.angular_velocity.z = self.read_raw_data(GYRO_ZOUT_H) / 131.0

        self.imu_publisher.publish(msg)
        self.get_logger().info('publish msg: {0}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('keyboard Interrupt')
    finally:
         node.destroy_node()
         rclpy.shutdown()

if __name__ == '__main__':
    main()
