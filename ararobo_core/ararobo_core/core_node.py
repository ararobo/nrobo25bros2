import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from std_msgs.msg import String

class core_node(Node):
    def __init__(self):
        super().__init__('core_node')
        self.get_logger().info('CoreNode has been initialized.')
        self.box = self.create_subscription(Marker,"/box/box",self.box_callback,10)
        self.box_select = self.create_publisher(Marker,"/box/boxselect",10)
        self.goal = self.create_publisher(Pose2D,"goal_pose",10)
        self.controller = self.create_subscription(String,"/controller",self.controller_callback,10)
        #self.timer = self.create_timer(0.5, self.controller_callback)
        self.timer_goal = self.create_timer(0.5, self.goal_timer_callback)
        #self.timer_box = self.create_timer(0.5, self.box_callback)

        self.box_coller = 0
        self.team = "red"
        self.startart_x = 0.0
        self.startart_y = 0.0
        self.g2_x = 0.0
        self.g2_y = 0.0
        self.g3_x = 0.0
        self.g3_y = 0.0
        self.gpub_x = -10000.0
        self.gpub_y = -10000.0
        self.serect_boxID = 0

    def controller_callback(self, msg: String):
        self.get_logger().info(f'Received contrpller message: {msg}')
        # Process the goal message and publish to nav/goal if needed
        self.cmd = msg
        cmds = msg.cmd.split(',')
        if msg.cmd == "start":
            self.gpub_x = self.startart_x
            self.gpub_y = self.startart_y
        elif msg.cmd == "g2":
            self.gpub_x = self.g2_x
            self.gpub_y = self.g2_y
        elif msg.cmd == "g3":
            self.gpub_x = self.g3_x
            self.gpub_y = self.g3_y
        elif cmds[0] == "box":
            if cmds[1] == "1":
                self.serect_boxID = 1
            elif cmds[1] == "2":
                self.serect_boxID = 2
            elif cmds[1] == "3":
                self.serect_boxID = 3
            elif cmds[1] == "4":
                self.serect_boxID = 4

    def goal_timer_callback(self):
        msg = Pose2D()
        msg.x = self.gpub_x
        msg.y = self.gpub_y
        if msg.x != -10000.0 and msg.y != -10000.0:
            self.goal.publish(msg)
            self.get_logger().info(f'Published_goal: x={msg.x}, y={msg.y}')
        else:
            self.get_logger().warn('No valid goal to publish, skipping.')
        self.gpub_x = -10000.0
        self.gpub_y = -10000.0
                
    def box_callback(self, msg):
        self.get_logger().info(f'Received box message: {msg}')
        # Process the box message and publish to box/boxselect if needed
        self.box_coller = msg.id
        

        self.goal.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = core_node()
    rclpy.spin(node)
    rclpy.shutdown()