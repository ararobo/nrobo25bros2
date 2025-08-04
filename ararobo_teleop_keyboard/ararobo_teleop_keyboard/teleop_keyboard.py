import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from ararobo_msgs.msg import MdData

LINEAR_SPEED = 0.3  # m/s (forward/backward)
STRAFE_SPEED = 0.3  # m/s (left/right strafe)
ANGULAR_SPEED = 1.0 # rad/s (rotation)
LIFT_VELOCITY = 30.0  # rad/s (lift speed) max:40
ARM_VELOCITY = 1.0  # m/s (arm speed)
WIDTH_VELOCITY = 1.0  # m/s (width adjustment speed)
TIMEOUT = 0.05       # seconds (timeout for key press detection)

KEY_MAPPINGS = {
    'w': ( 0.0,  1.0,  0.0), # Forward (linear.y +)
    's': (0.0,  -1.0,  0.0), # Backward (linear.y -)
    'a': ( 1.0,  0.0,  0.0), # Strafe Left (linear.x +)
    'd': (-1.0,  0.0,  0.0), # Strafe Right (linear.x -)
    '\x1b[C': ( 0.0,  0.0, -1.0), # Right Arrow (angular.z -)
    '\x1b[D': ( 0.0,  0.0,  1.0), # Left Arrow (angular.z +)
}

# Instructions message
INSTRUCTIONS = """
Reading from keyboard!
---------------------------
Control the Omni Robot:
  w: Move Forward
  s: Move Backward
  a: Strafe Left
  d: Strafe Right
  right: Rotate Right
  left: Rotate Left

Control the lift
  up: Lift Up
  down: Lift Down
  
Control the hand
  t: Decrease Left Low Width
  g: Increase Left Low Width
  f: Move Left Low Arm Down
  h: Move Left Low Arm Up
  i: Decrease Right Low Width
  k: Increase Right Low Width
  j: Move Right Low Arm Down
  l: Move Right Low Arm Up

Ctrl-C to quit
"""

class TeleopKeyboard(Node):
    """ROS 2 node for keyboard teleoperation."""
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub_cmd_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_lift_ = self.create_publisher(Float32, '/lift_vel', 10)
        self.pub_md_data_ = self.create_publisher(MdData, '/md_data', 10)
        self.get_logger().info("Teleop Keyboard node started.")

        # Save original terminal settings
        self.original_settings = termios.tcgetattr(sys.stdin)

        # Variables to hold desired velocities
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

    def print_instructions(self):
        """Prints the control instructions."""
        self.get_logger().info(INSTRUCTIONS)

    def restore_settings(self):
        """Restores original terminal settings."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
        self.get_logger().info("\nTerminal settings restored.")

    def run(self):
        """Main loop for reading keys and publishing Twist messages."""
        self.print_instructions()

        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())

            while rclpy.ok():
                # Check for key press with a timeout
                rlist, _, _ = select.select([sys.stdin], [], [], TIMEOUT)

                if rlist:
                    key = sys.stdin.read(1)
                    if key == '\x1b':  # Arrow key sequence
                        key += sys.stdin.read(2)
                else:
                    key = ''

                # Reset velocities to zero (default state if no key pressed)
                target_linear_x = 0.0
                target_linear_y = 0.0
                target_angular_z = 0.0
                
                target_lift_vel = 0.0  # Default lift velocity
                
                target_ll = 0.0  # Default low left
                target_lr = 0.0  # Default low right
                target_ul = 0.0  # Default upper left
                target_ur = 0.0  # Default upper right
                target_ll_w = 0.0  # Default low left width
                target_lr_w = 0.0  # Default low right width
                target_ul_w = 0.0  # Default upper left width
                target_ur_w = 0.0  # Default upper right width

                if key:
                    if key in KEY_MAPPINGS:
                        # Map the key to the corresponding velocity direction
                        lx, ly, az = KEY_MAPPINGS[key]
                        target_linear_x = lx * LINEAR_SPEED
                        target_linear_y = ly * STRAFE_SPEED
                        target_angular_z = az * ANGULAR_SPEED
                    elif key == '\x1b[A':  # Up Arrow
                        target_lift_vel = LIFT_VELOCITY  # Lift up
                    elif key == '\x1b[B':  # Down Arrow
                        target_lift_vel = -LIFT_VELOCITY
                    elif key == 't':
                        target_ul_w = -WIDTH_VELOCITY  # Decrease left low width
                    elif key == 'g':
                        target_ul_w = WIDTH_VELOCITY
                    elif key == 'f':
                        target_ul = -ARM_VELOCITY  # Move left low arm down
                    elif key == 'h':
                        target_ul = ARM_VELOCITY
                    elif key == 'i':
                        target_ur_w = -WIDTH_VELOCITY
                    elif key == 'k':
                        target_ur_w = WIDTH_VELOCITY
                    elif key == 'j':
                        target_ur = -ARM_VELOCITY
                    elif key == 'l':
                        target_ur = ARM_VELOCITY
                    elif key == '\x03':  # Ctrl+C
                        self.get_logger().info("Ctrl+C detected. Shutting down.")
                        break
                    else:
                        pass
                else:
                    pass

                self.linear_x = target_linear_x
                self.linear_y = target_linear_y
                self.angular_z = target_angular_z

                # Create and publish the Twist message
                twist = Twist()
                twist.linear.x = self.linear_x
                twist.linear.y = self.linear_y
                twist.linear.z = 0.0  # Omni doesn't typically use vertical linear motion
                twist.angular.x = 0.0  # Omni doesn't typically use roll/pitch
                twist.angular.y = 0.0
                twist.angular.z = self.angular_z
                self.pub_cmd_.publish(twist)
                
                # Create and publish the Float32 message for lift velocity
                lift_msg = Float32()
                lift_msg.data = target_lift_vel
                self.pub_lift_.publish(lift_msg)
                
                # Create and publish the MdData message for arm and width control
                md_data_msg = MdData()
                md_data_msg.ll = target_ll
                md_data_msg.lr = target_lr
                md_data_msg.ul = target_ul
                md_data_msg.ur = target_ur
                md_data_msg.ll_w = target_ll_w
                md_data_msg.lr_w = target_lr_w
                md_data_msg.ul_w = target_ul_w
                md_data_msg.ur_w = target_ur_w
                self.pub_md_data_.publish(md_data_msg)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            # Ensure terminal settings are restored even on error
            self.restore_settings()


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopKeyboard()
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass # Handled by Ctrl+C check inside run loop
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()