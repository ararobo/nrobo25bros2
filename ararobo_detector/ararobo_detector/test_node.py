import rclpy
from rclpy.node import Node

class test_node(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Hello World!')

def main(args=None):
    rclpy.init(args=args)
    node = test_node()
    rclpy.spin(node)
    rclpy.shutdown()