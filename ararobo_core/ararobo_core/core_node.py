import rclpy
from rclpy.node import Node

class core_node(Node):
    def __init__(self):
        super().__init__('core_node')
        self.get_logger().info('CoreNode has been initialized.')
        self.box = self.create_subscription(Twist,"/box/box",10)
        self.box_select = self.create_subscription(Twist,"/box/boxselect",10)
        self.goal = self.create_publisher(Twist,"/nav/goal",10)
        self.decbox = self.create_publisher(Twist,"/box/",10)

def main(args=None):
    rclpy.init(args=args)
    node = core_node()
    rclpy.spin(node)
    rclpy.shutdown()