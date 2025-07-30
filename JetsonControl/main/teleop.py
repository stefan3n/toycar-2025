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
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info("CmdVelListener started, connected to Arduino on /dev/ttyACM0")

    def send_command(self, pwmR, pwmL, revR, revL):
        # Construim string-ul comenzii in formatul Arduino
        cmd_str = f"<0,2,{pwmR},{pwmL},{int(revR)},{int(revL)}>"
        self.serial_port.write(cmd_str.encode())
        self.get_logger().info(f"Sent command to Arduino: {cmd_str}")

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        pwm = 100  # ajustează viteza între 0-255 după necesitate

        if linear > 0.1:
            # Mers înainte
            self.send_command(pwm, pwm, False, False)
        elif linear < -0.1:
            # Mers înapoi
            self.send_command(pwm, pwm, True, True)
        elif angular > 0.1:
            # Viraj stânga
            self.send_command(pwm, pwm, False, True)
        elif angular < -0.1:
            # Viraj dreapta
            self.send_command(pwm, pwm, True, False)
        else:
            # Stop
            self.send_command(0, 0, False, False)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

