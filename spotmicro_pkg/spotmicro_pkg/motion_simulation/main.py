import rclpy
from spotmicro_pkg.motion_simulation.motion_simulation import MotionSimulation
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotionSimulation()
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

