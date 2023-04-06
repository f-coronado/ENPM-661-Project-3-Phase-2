import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from numpy import pi, cos, sin, sqrt
import time

# velocityList = [[(1, 2, 0), (2, 5, 0), (5, 5, 0), (5, 5, 0), (5, 5, 0), (5, 2, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)], \
#                 [(2, 2, 0), (2, 5, 0), (5, 5, 0), (5, 5, 0), (5, 5, 0), (5, 2, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)], \
#                 [(3, 5, 0), (2, 5, 0), (5, 5, 0), (5, 5, 0), (5, 5, 0), (5, 2, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)], \
#                 [(4, 5, 0), (2, 5, 0), (5, 5, 0), (5, 5, 0), (5, 5, 0), (5, 2, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)]]

# [rpm1, rpm2]
# velocityList = [[(2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)]]

# [0, rpm2]
velocityList = [[(0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0)]]

# [rpm1, rpm1]
# velocityList = [[(2, 2, 0), (2, 2, 0), (2, 2, 0), (2, 2, 0), (2, 2, 0), (2, 2, 0), (2, 2, 0), (2, 2, 0), (2, 2, 0), (2, 2, 0)]]

# [0, rpm1]
# velocityList = [[(0, 2, 0), (0, 2, 0), (0, 2, 0), (0, 2, 0), (0, 2, 0), (0, 2, 0), (0, 2, 0), (0, 2, 0), (0, 2, 0), (0, 2, 0)]]

def publishVelocities(velocityList):

    msg = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    t = 0
    dt = .1 # seconds
    r = .033 # meters
    L = .16 # meters
    rospy.init_node('robot_talker', anonymous=True)
    rate = rospy.Rate(1/.1) # should be freq from astar cost fn, (1/dt for every intermediate), or 1s if we save vel
    i = 0
    rowNumber = 1
    numberOfRows = len(velocityList)
    linearFactorStraight = 1.4459225
    linearFactorCurved = 16.939258
    # angularFactor = .02110842
    angularFactor = 4.3190914 * 2.2789121 * 1.065
 # subscribe to odom topic, at each point create an x, y and theta

    while not rospy.is_shutdown():

        for row in velocityList:
            print("\nrow: ", row)

            for velocities in row:
                UL = velocities[0]
                UR = velocities[1]
                theta = velocities[2]
                thetaN = pi * theta / 180

                if t <= 1:

                    print("t = ", t, "on iteration: ", i)
                    # print()
                            # (m / m) * (rpm) * (2pi rad/1rpm) * (1min/60s) * s
                    thetaN += (r / L) * (UR - UL) * 2*pi / 60 * dt # rad
                    
                                    # (m / m) * (rpm - rpm) * (1rad/1rpm) * cos
                    # msg.linear.x = (r / 2) * (UL + UR) * 2*pi * cos(thetaN) # m/s check units
                    # msg.linear.y = (r / 2) * (UL + UR) * 2*pi * sin(thetaN)
                    Vx = (r / 2) * (UL + UR) * 2*pi/60 * cos(thetaN) # m/s check units
                    Vy = (r / 2) * (UL + UR) * 2*pi/60 * sin(thetaN)
                    # if UL == UR:
                    print("UL == UR")
                    msg.linear.x = sqrt(Vx**2 + Vy**2) * linearFactorStraight
                    msg.angular.z = (r / L) * (UR - UL) * 2*pi/60 * angularFactor # radians/s
                        # multiplying by factor bc motors and encoders? are iffy. helpful to upscale angular.z by constant, look at odom and compare
                    # if UL != UR:
                    #     print("UL != UR")
                    #     msg.linear.x = sqrt(Vx**2 + Vy**2) * linearFactorCurved
                    #     msg.angular.z = (r / L) * (UR - UL) * 2*pi/60 * angularFactor

                    pub.publish(msg)
                    rate.sleep()
                
                t = round(t + dt, 2)

                if t >= 1:
                    t = 0
                    print("\nt > 1, stopping robot, moving onto next row")
                    msg.angular.z = 0
                    msg.linear.x = 0
                    pub.publish(msg)
                    break
                
                i += 1

            if int(rowNumber) >= numberOfRows:
                print("\nending script")
                return
            rowNumber += 1
            # print('rowNumber: ', rowNumber, "t: ", t)

if __name__=='__main__':
    try:
        publishVelocities(velocityList)
    except rospy.ROSInternalException:
        pass