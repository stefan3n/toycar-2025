import rospy
from sensor_msgs.msg import LaserScan
from math import pi, sqrt

globalPipe = None

def fromRotToPWM(rot):
    return 255 * rot / (650 / 60) 


def obstacle_avoider_process(pipe,logger=None):
    import rospy
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist

    print(" obstacle_avoider_process PORNIT")

    def callback(scan):
        front_ranges = scan.ranges[0:30] + scan.ranges[-30:]
        dists = [d for d in front_ranges if d > 0.05]

        if not dists:
            return

        min_dist = min(dists)
        print(f" Dist min in fata: {min_dist:.2f} m")
        if min_dist > 1.0:
        
            pwm = 30
            msg = [0, 4, pwm, pwm, 0, 0]  # SET_PWM
        else:
            
            pwm = 30
            msg = [0, 4, pwm, pwm, 0, 1]  # SET_PWM

        print("Trimitem la motoare:", msg)
        pipe.send(msg)

    rospy.init_node("obstacle_avoider")
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()