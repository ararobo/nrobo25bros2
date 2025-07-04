import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

class core_node(Node):
    def __init__(self):
        super().__init__('core_node')
        self.get_logger().info('CoreNode has been initialized.')
        self.box = self.create_subscription(Twist,"/box/box",10)
        self.box_select = self.create_publisher(Twist,"/box/boxselect",10)
        self.goal = self.create_publisher(Pose2D,"/nav/goal",10)
        self.controller = self.create_subscription(Bool,"/controller",10)

        self.box_coller = 0
        self.team = "red"
        self.startarea = 0.0
        self.g2 = 0.0
        self.g3 = 0.0

    def controller_callback(self, msg):
        self.get_logger().info(f'Received contrpller message: {msg}')
        # Process the goal message and publish to nav/goal if needed
        self.cmd = msg
        cmds = msg.cmd.split(',')
        if msg.cmd == "start":
            msg.data = f"{self.startarea}"
        elif msg.cmd == "g2":
            msg.data = f"{self.g2}"
        elif msg.cmd == "g3":
            msg.data = f"{self.g3}"
        elif cmds[0] == "box":
            if cmds[]

        self.goal.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = core_node()
    rclpy.spin(node)
    rclpy.shutdown()