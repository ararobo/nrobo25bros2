import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.3  # m/s (forward/backward)
STRAFE_SPEED = 0.3  # m/s (left/right strafe)
ANGULAR_SPEED = 0.5 # rad/s (rotation)
TIMEOUT = 0.1       # seconds (timeout for key press detection)

KEY_MAPPINGS = {
    'w': ( 1.0,  0.0,  0.0), # Forward (linear.x +)
    's': (-1.0,  0.0,  0.0), # Backward (linear.x -)
    'a': ( 0.0,  1.0,  0.0), # Strafe Left (linear.y +)
    'd': ( 0.0, -1.0,  0.0), # Strafe Right (linear.y -)
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
  ->: Rotate Right
  <-: Rotate Left

Ctrl-C to quit
"""

def get_key(settings, timeout):
    """Reads a single character from stdin with a timeout."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b': # Escape key starts sequence
             key += sys.stdin.read(2) # Read next two characters
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopKeyboard(Node):
    """ROS 2 node for keyboard teleoperation."""
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/robot/move', 10)
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
            while rclpy.ok():
                key = get_key(self.original_settings, TIMEOUT)

                # Reset velocities to zero (default state if no key pressed)
                target_linear_x = 0.0
                target_linear_y = 0.0
                target_angular_z = 0.0

                if key:
                    if key in KEY_MAPPINGS:
                        # Map the key to the corresponding velocity direction
                        lx, ly, az = KEY_MAPPINGS[key]
                        target_linear_x = lx * LINEAR_SPEED
                        target_linear_y = ly * STRAFE_SPEED
                        target_angular_z = az * ANGULAR_SPEED
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
                twist.linear.z = 0.0 # Omni doesn't typically use vertical linear motion
                twist.angular.x = 0.0 # Omni doesn't typically use roll/pitch
                twist.angular.y = 0.0
                twist.angular.z = self.angular_z

                self.publisher_.publish(twist)

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