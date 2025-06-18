import rclpy
from rclpy.node import Node

class core_node(Node):
    def __init__(self):
        super().__init__('core_node')
        self.get_logger().info('CoreNode has been initialized.')

def main(args=None):
    rclpy.init(args=args)
    node = core_node()
    rclpy.spin(node)
    rclpy.shutdown()