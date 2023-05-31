import rclpy
from spotmicro_pkg.communication.communication import Communication

def main(args=None):
    rclpy.init()
    node = Communication()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
