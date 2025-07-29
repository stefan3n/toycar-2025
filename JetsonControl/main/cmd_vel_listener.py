import rospy
from geometry_msgs.msg import Twist
from serial import Serial

def cmd_vel_listener():
    rospy.init_node("cmd_vel_listener", anonymous=True)
    serial_port = Serial("/dev/ttyACM0", 115200, timeout=1)
    rospy.Subscriber("/cmd_vel", Twist, lambda msg: cmd_vel_callback(msg, serial_port))
    rospy.spin()

def cmd_vel_callback(msg, serial_port):
    linear = msg.linear.x  # Viteza liniară (m/s)
    angular = msg.angular.z  # Viteza unghiulară (rad/s)
    pwm = 20  # Valoare PWM pentru Car.AUTO_PWM
    if linear > 0.1:
        command = f"<0,2,{pwm},{pwm},0,1>"  # Forward
    elif linear < -0.1:
        command = f"<0,2,-{pwm},-{pwm},0,1>"  # Backward
    elif angular > 0.1:
        command = f"<0,2,-{pwm},{pwm},0,1>"  # Turn left
    elif angular < -0.1:
        command = f"<0,2,{pwm},-{pwm},0,1>"  # Turn right
    else:
        command = "<0,2,0,0,0,1>"  # Stop
    serial_port.write(command.encode())
    print(f"Sent to motors: {command}")