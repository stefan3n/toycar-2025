import rclpy
from rclpy.node import Node
import sys
import tty
import termios
import select
import serial

class KeyboardSerialTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_serial_teleop')
        self.get_logger().info('A pornit serialul.')

    
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.pwm_max = 30
        self.pwm_min = 20

    def cap_pwm(self, val):
        val = int(abs(val))
        return max(self.pwm_min, min(self.pwm_max, val))

    def timer_callback(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)

            pwmL, pwmR = 0, 0
            dirL, dirR = 1, 1

            if key == 'q':
                self.get_logger().info('Iesire.')
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                rclpy.shutdown()
                return

            elif key == '\x1b':
                next1, next2 = sys.stdin.read(2)
                if next1 == '[':
                    if next2 == 'W':  # sus
                        pwmL = pwmR = self.pwm_max
                        dirL = 1
                        dirR = 1
                    elif next2 == 'S':  
                        pwmL = pwmR = self.pwm_max
                        dirL = 0
                        dirR = 0
                    elif next2 == 'A':  
                        pwmL = pwmR = self.pwm_max
                        dirL = 0
                        dirR = 1
                    elif next2 == 'D': 
                        pwmL = pwmR = self.pwm_max
                        dirL = 1
                        dirR = 0
                    else:
                        return  
                else:
                    return  
            else:
               
                pwmL = pwmR = 0
                dirL = dirR = 1

            command = f"<0,2,{pwmR},{pwmL},{dirR},{dirL}>"
            self.serial_port.write(command.encode())
            self.get_logger().info(f'Sent command: {command}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSerialTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
