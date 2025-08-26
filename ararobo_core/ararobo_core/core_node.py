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
        self.pylon_x = 20.0
        self.pylon_y = 20.0
        self.share_B_x = 30.0
        self.share_B_y = 30.0
        self.share_C_x = 40.0
        self.share_C_y = 40.0
        self.workspease_x = 50.0
        self.workspease_y = 50.0
        self.cereategate_x = 60.0
        self.cereategate_y = 60.0
        self.proprietary_start_x = 10.0
        self.proprietary_start_y = 10.0
        self.proprietary_B_x = 70.0
        self.proprietary_B_y = 70.0
        self.proprietary_C_x = 80.0
        self.proprietary_C_y = 80.0
        self.trolleyconect_x = 90.0
        self.trolleyconect_y = 90.0
        self.gpub_x = -10000.0
        self.gpub_y = -10000.0
        self.serect_boxID = 0
        self.armpose = 1

    def controller_callback(self, msg: String):
        self.get_logger().info(f'Received contrpller message: {msg}')
        # Process the goal message and publish to nav/goal if needed
        self.cmd = msg.data
        cmds = msg.data.split(',')
        if cmds[0] == "move":
            if cmds[1] == "start":
                self.gpub_x = self.startart_x
                self.gpub_y = self.startart_y
            elif cmds[1] == "pylon":
                self.gpub_x = self.pylon_x
                self.gpub_y = self.pylon_y
            elif cmds[1] == "shareB":
                self.gpub_x = self.share_B_x
                self.gpub_y = self.share_B_y
            elif cmds[1] == "shareC":
                self.gpub_x = self.share_C_x
                self.gpub_y = self.share_C_y
            elif cmds[1] == "workspease":
                self.gpub_x = self.workspease_x
                self.gpub_y = self.workspease_y
            elif cmds[1] == "cereategate":
                self.gpub_x = self.cereategate_x
                self.gpub_y = self.cereategate_y
            elif cmds[1] == "proprietary_start":
                self.gpub_x = self.proprietary_start_x
                self.gpub_y = self.proprietary_start_y
            elif cmds[1] == "proprietary_B":
                self.gpub_x = self.proprietary_B_x
                self.gpub_y = self.proprietary_B_y
            elif cmds[1] == "proprietary_C":
                self.gpub_x = self.proprietary_C_x
                self.gpub_y = self.proprietary_C_y
            elif cmds[1] == "trolleyconect":
                self.gpub_x = self.trolleyconect_x
                self.gpub_y = self.trolleyconect_y
        elif cmds[0] == "box":
            if cmds[1] == "1":
                self.serect_boxID = 1
            elif cmds[1] == "2":
                self.serect_boxID = 2
            elif cmds[1] == "3":
                self.serect_boxID = 3
            elif cmds[1] == "4":
                self.serect_boxID = 4
        elif cmds[0] == "armpose":
            if cmds[1] == "close":
                self.armpose = 0
            elif cmds[1] == "open":
                self.armpose = 1
            elif cmds[1] == "catch":
                self.armpose = 2

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