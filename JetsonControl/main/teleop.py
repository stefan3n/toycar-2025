import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Keyboard teleop started. Use arrows to move. Press q to quit.')

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            twist = Twist()

            if key == 'q':
                self.get_logger().info('Quitting...')
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                rclpy.shutdown()

            elif key == '\x1b':
                next1, next2 = sys.stdin.read(2)
                if next1 == '[':
                    if next2 == 'A':  # Up arrow
                        twist.linear.x = 1.0
                    elif next2 == 'B':  # Down arrow
                        twist.linear.x = -1.0
                    elif next2 == 'C':  # Right arrow
                        twist.angular.z = -1.0
                    elif next2 == 'D':  # Left arrow
                        twist.angular.z = 1.0

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
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
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.pwm_max = 30
        self.pwm_min = 20

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        pwmL, pwmR = 0, 0
        dirL, dirR = 1, 1

        def cap_pwm(val):
            val = int(abs(val))
            return max(self.pwm_min, min(self.pwm_max, val))

        if abs(linear) > 0.1:
            pwmL = pwmR = cap_pwm(linear * 100)
            dirL = dirR = 1 if linear > 0 else 0

        elif abs(angular) > 0.1:
            pwm = cap_pwm(angular * 100)
            pwmL = pwmR = pwm
            if angular > 0:
                dirL = 1
                dirR = 0  
            else:
                dirL = 0
                dirR = 1  

        else:
            pwmL = pwmR = 0
            dirL = dirR = 1

        cmd = f"<0,2,{pwmR},{pwmL},{dirR},{dirL}>"
        self.serial_port.write(cmd.encode())
        self.get_logger().info(f'Trimis la Arduino: {cmd}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
