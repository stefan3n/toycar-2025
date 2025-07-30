import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import termios
import tty
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop ready. W/A/S/D to move. Q to quit.')
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            twist = Twist()

            if key == 'w':
                twist.linear.x = 1.0
            elif key == 's':
                twist.linear.x = -1.0
            elif key == 'a':
                twist.angular.z = 1.0
            elif key == 'd':
                twist.angular.z = -1.0
            elif key == 'q':
                self.get_logger().info("Quit pressed. Exiting...")
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                rclpy.shutdown()
                return

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
