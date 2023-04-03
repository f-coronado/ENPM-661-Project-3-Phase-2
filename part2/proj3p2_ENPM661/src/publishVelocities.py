import rospy
from geometry_msgs.msg import Twist
from numpy import pi, cos, sin
import time

def publishVelocities(UL, UR, theta):
    msg = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    t = float(0)
    dt = .1
    r = .105 # mm
    L = .16
    thetaN = pi * theta / 180
    rospy.init_node('robot_talker', anonymous=True)

    while not rospy.is_shutdown():
        if t < 1:
            # if UL != 0 and UR == 0:
            #         print("t = ", t)
            #         # angularVelocityLeft = 2 * pi * UL / 60
            #         # angularVelocityRight = 2 * pi * UL / 60
            #         # msg.angular.z = (angularVelocityLeft + angularVelocityRight)/2
            #         msg.angular.z = (r/L)*(UL) * dt
            #         msg.linear.x = 0
            #         pub.publish(msg)
            #         time.sleep(.1)
            
            # if UL == 0 and UR != 0:
            #     if t < 1:
            #         print("t = ", t)
            #         # angularVelocityLeft = 2 * pi * UL / 60
            #         # angularVelocityRight = 2 * pi * UL / 60
            #         # msg.angular.z = (angularVelocityLeft + angularVelocityRight)/2
            #         msg.angular.z = (r/L)*(-UR) * dt
            #         msg.linear.x = 0
            #         pub.publish(msg)
            #         time.sleep(.1)

            # if UL == UR:
            #     if t < 1:
            #         print("t = ", t)
            #         # angularVelocityLeft = 2 * pi * UL / 60
            #         # angularVelocityRight = 2 * pi * UL / 60
            #         # msg.angular.z = (angularVelocityLeft + angularVelocityRight)/2
            #         msg.angular.z = 0
            #         msg.linear.x = (r/ 2) * (UL + UR) * cos(theta) * dt
            #         pub.publish(msg)
            #         time.sleep(.1)

            if UL < UR:
                print("UL < UR")
                print("t = ", t)
                speedDifference = UR - UL
                thetaN += (r / L) * (UR - UL) * dt
                msg.linear.x = (r / 2) * (UL + UR) * cos(thetaN) * dt
                # msg.linear.y = (r / 2) * (UL + UR) * sin(thetaN) * dt
                msg.angular.z = (r/L) * (UR - UL)
                # msg.linear.x = (UL + UR) / 2
                msg.linear.y = 0
                # msg.angular.z = speedDifference * 0.1
                pub.publish(msg)
                time.sleep(.1)

        t += dt

        if t > 1:
            print("t > 1")
            msg.angular.z = 0
            msg.linear.x = 0
            pub.publish(msg)
            break

if __name__=='__main__':
    try:
        publishVelocities(2, 5, 30)
    except rospy.ROSInternalException:
        pass