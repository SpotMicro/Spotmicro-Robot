from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from std_msgs.msg import String


class Communication(Node):
    def __init__(self):
        super().__init__('communication_node')
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.keyboard_input_publisher = self.create_publisher(String, 'keyboard_input', QOS_RKL10V)
        self.timer = self.create_timer(0.1, self.publish_keyboard_input_msg)

    def publish_keyboard_input_msg(self):
        msg = String()
        msg.data = input()
        self.keyboard_input_publisher.publish(msg)
        self.get_logger().info('publish msg: {0}'.format(msg.data))
