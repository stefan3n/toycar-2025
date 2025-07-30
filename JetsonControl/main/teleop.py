import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Use arrow keys to move. Press q to quit.')

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            twist = Twist()

            if key == 'q':
                self.get_logger().info('Exiting...')
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                rclpy.shutdown()
                return

            elif key == '\x1b':
                if sys.stdin.read(1) == '[':
                    direction = sys.stdin.read(1)
                    if direction == 'A':
                        twist.linear.x = 1.0
                    elif direction == 'B':
                        twist.linear.x = -1.0
                    elif direction == 'C':
                        twist.angular.z = -1.0
                    elif direction == 'D':
                        twist.angular.z = 1.0

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info('Listening to /cmd_vel and sending to Arduino.')

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        pwm = 30

        if linear > 0.1:
            command = f"<0,2,{pwm},{pwm},0,1>"
        elif linear < -0.1:
            command = f"<0,2,-{pwm},-{pwm},0,1>"
        elif angular > 0.1:
            command = f"<0,2,-{pwm},{pwm},0,1>"
        elif angular < -0.1:
            command = f"<0,2,{pwm},-{pwm},0,1>"
        else:
            command = "<0,2,0,0,0,1>"

        self.serial.write(command.encode())
        self.get_logger().info(f"Sent: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
