import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from numpy import pi, cos, sin, sqrt
import time

import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from queue import PriorityQueue
import math

boundry = []
Pth = {}        #Stores the path for backtracking
rpmDict = {}
UncheckedList = PriorityQueue()     #Used to store unvisited nodes
b_track = []            
CloseList = []
CheckedList = np.zeros((250,600),dtype='uint8')     # Used to store the visited nodes
Robot_Radius = 11     #Needs to be changed to 10.5
Wheel_Radius = 3.3
Wheel_Length = 16
#Creating the Obstacle Space
#Obtacle with Obstacle, Obstacle Clearance and Robot Radius
def obstacle_space(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if ((200-l) - 5 < 0) or ((m) - 5 < 0) or ((200-l) -195 > 0) or ((m) - 595 > 0): #boundary
                space[l][m] = [0,0,255]
                
            if (m > ((150-Obstacle_Clearance)-Robot_Radius)) and (m < (165+Obstacle_Clearance+Robot_Radius)) and (200-l < 200) and (200-l > ((75-Obstacle_Clearance)-Robot_Radius)):   #rectangle1 with Obstacle clearance and Robot Radius
                space[l][m] = [20,0,0]

            if (m > (150-Obstacle_Clearance)) and (m < (165+Obstacle_Clearance)) and (200-l < 200) and (200-l > (75-Obstacle_Clearance)):   #rectangle1 with Obstacle Clearance
                space[l][m] = [0,255,0]
                
            if (m > 150) and (m < 165) and (200-l < 200) and (200-l > 75):   #rectangle1
                space[l][m] = [0,0,255]
                
            if (m > ((250-Obstacle_Clearance)-Robot_Radius)) and (m < (265+Obstacle_Clearance+Robot_Radius)) and (200-l >0) and (200-l < (125+Obstacle_Clearance+Robot_Radius)):  #rectangle2 with Obstacle clearance and Robot Radius
                space[l][m] = [20,0,0]

            if (m > (250-Obstacle_Clearance)) and (m < (265+Obstacle_Clearance)) and (200-l >0) and (200-l < (125+Obstacle_Clearance)):  #rectangle2 with Obstacle clearance
                space[l][m] = [0,255,0]

            if (m > 250) and (m < 265) and (200-l >0) and (200-l < 125):  #rectangle2
                space[l][m] = [0,0,255]
            
            if (math.pow((m-400),2) + math.pow(((200-l)-110),2) - math.pow((50+Obstacle_Clearance+Robot_Radius),2)) < 0:    #Circle with Obstacle clearance and Robot Radius
                space[l][m] = [20,0,0]

            if (math.pow((m-400),2) + math.pow(((200-l)-110),2) - math.pow((50+Obstacle_Clearance),2)) < 0: #Circle with Obstacle Clearance
                space[l][m] = [0,255,0]
                
            if (math.pow((m-400),2) + math.pow(((200-l)-110),2) - math.pow(50,2)) < 0: #Circle
                space[l][m] = [0,0,255]                
    return 
#taking the obstacle coordinates into a list
def boundry_creation(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if space[l][m][2] == 255:
                boundry.append((m,250-l))
            if space[l][m][1] == 255:
                boundry.append((m,250-l))
            if space[l][m][0] == 20:
                boundry.append((m,250-l))
    return boundry




#Getting User Inputs For the Start node from the user

# def PromptPredefined():
#     print("Want to use predefined start, goal and rpms?")
#     ans = input("y/n?")
#     if ans == "y":
#         Obstacle_Clearance = 1
#         obstacle_space(space)
#         Obs_Coords= boundry_creation(space)
#         xStart = 10
#         yStart = 10
#         zStart = 45
#         startNode = (10, 10, 45)
#         xGoal = 100
#         yGoal = 30
#         goalNode = ()
#         rpm1 = 2
#         rpm2 = 5

#         return True
#     else:
#         return False
def User_Inputs_Start(Obs_Coords):
    while True:
        x = int(input("Enter the Initial x node: "))
        y = int(input("Enter the Initial y node: "))
        z = int(input("Enter the orientation of robot at start pt.(in degrees): "))
        
        if((x>=0) or (x<=600)) and (y>=0) or (y<=250):
            if (x,y) not in Obs_Coords :
                start_node = (x,y,z)
                return start_node
            else:
                print("The Entered Start Node is in obstacle space")
#Getting User Input for the Goal Node from the user
def User_Inputs_Goal(Obs_Coords):
    while True:
        x = int(input("Enter the Goal x node: "))
        y = int(input("Enter the Goal y node: "))
        
        #goal_node = (x,y)
        if((x>=0) or (x<=600)) and (y>=0) or (y<=250):
            if (x,y) not in Obs_Coords :
                goal_node=(x,y)
                break
            else:
                print("The Entered Goal Node is in obstacle space")
    return goal_node

def User_Input_rpm():
    rpm1 = int(input("Enter the first RPM: "))
    rpm2 = int(input("Enter the second RPM: "))
    
    return (rpm1,rpm2)
def angle_conversion(theta):
    if theta > 360:
        theta = theta % 360
        return theta
    elif theta < -360:
        theta = (-theta % 360)*(-1) 
        return theta
    else:
        return theta
    
fig, ax = plt.subplots()

#Converting the rpm to velocity
# def rpm_to_velocity(rpm1,rpm2):
#     v1 = (2*np.pi*Wheel_Radius*rpm1)/60
#     v2 = (2*np.pi*Wheel_Radius*rpm2)/60
#     return v1,v2

def func_Cost(old_x,old_y,theta,ul,ur):
    rpmList = []
    t = 0
    dt = 0.1
    theta_n = (np.pi * (theta/180))
    D = 0
    Xn = old_x
    Yn = old_y
    while t<1:
        rpmLeft = (ul * 60) / (2 * np.pi*Wheel_Radius)
        rpmRight = (ur * 60) / (2 * np.pi*Wheel_Radius)
        rpmList += {(round(rpmLeft, 3), round(rpmRight, 3), round(theta_n, 3))}
        # print(rpmList)
        t = t+dt
        Xn += 0.5*Wheel_Radius*(ul+ur)*np.cos(theta_n)*dt
        Yn += 0.5*Wheel_Radius*(ul+ur)*np.sin(theta_n)*dt
        # print(Xn)
        # print(Yn)
        theta_n += (Wheel_Radius/Wheel_Length)*(ur-ul)
        if (round(Xn),round(Yn)) not in Obs_Coords:
            D = D+math.sqrt(math.pow((0.5*Wheel_Radius * (ul + ur) * np.cos(theta_n) * dt),2) + math.pow((0.5*Wheel_Radius * (ul + ur) * np.sin(theta_n) * dt),2))
            
        else:
            D = 0
            Xn = old_x
            Yn = old_y
            return (D,Xn,Yn,theta_n)
    theta_n = (180*(theta_n))/np.pi
    theta_n = angle_conversion(theta_n)
    return (D, Xn, Yn, theta_n, rpmList)

def a_star_function(pos,vel1,vel2):
    Costfn = func_Cost(pos[0],pos[1],pos[2],vel1,vel2)
    if Costfn[0] !=0:
        newPos = (Costfn[1],Costfn[2],Costfn[3])
        childrpmList = Costfn[4]
        Cost = Costfn[0]
        x,y,_ = newPos
        if (0<=round(x)<600) and (0<=round(y)<250):
            if (CheckedList[int(round(y))][int(round(x))] != 1) and ((int(round(x)),int(round(y))) not in Obs_Coords):
                    CloseList.append((round(x),round(y),newPos[2]))
                    Eucledian_dist = np.sqrt(((goal_pt[0] - newPos[0])**2)+((goal_pt[1] - newPos[1])**2))
                    TotalCost = Cost + Eucledian_dist
                    for m in range(UncheckedList.qsize()):
                        if UncheckedList.queue[m][3] == newPos:
                            if UncheckedList.queue[m][0] > TotalCost:
                                UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,newPos)
                                Pth[(round(newPos[0]),round(newPos[1]),round(newPos[2]))] = (round(pos[0]),round(pos[1]),round(pos[2]))
                                rpmDict[(round(newPos[0]),round(newPos[1]),round(newPos[2]))] = childrpmList
                                return
                            else:
                                return
                    UncheckedList.put((TotalCost,Eucledian_dist,Cost,newPos))
                    Pth[(round(newPos[0]),round(newPos[1]),round(newPos[2]))] = (round(pos[0]),round(pos[1]),round(pos[2]))
                    rpmDict[(round(newPos[0]),round(newPos[1]),round(newPos[2]))] = childrpmList
                    

#left wheel rpm = 0 and right wheel rpm = rpm1
def zero_n_rpm1(a):
    pos = a[3]
    a_star_function(pos,0,vel_1)

#left wheel rpm = rpm1 and right wheel rpm = 0
def rpm1_n_zero(a):
    pos = a[3] 
    a_star_function(pos,vel_1,0)    
    
#left wheel rpm = rpm1 and right wheel rpm = rpm1
def rpm1_n_rpm1(a):
    pos = a[3]
    a_star_function(pos,vel_1,vel_1) 

# left wheel rpm = zero and right wheel rpm = rpm2
def zero_n_rpm2(a):
    pos = a[3]  
    a_star_function(pos,0,vel_2)
# left wheel rpm = rpm2 and right wheel rpm = zero
def rpm2_n_zero(a):
    pos = a[3]  
    a_star_function(pos,vel_2,0)

#left wheel rpm = rpm2 and right wheel rpm = rpm2
def rpm2_n_rpm2(a):
    pos = a[3] 
    a_star_function(pos,vel_2,vel_2)

#left wheel rpm = rpm1 and right wheel rpm = rpm2
def rpm1_n_rpm2(a):
    pos = a[3] 
    a_star_function(pos,vel_1,vel_2)
    
def rpm2_n_rpm1(a):
    pos = a[3] 
    a_star_function(pos,vel_2,vel_1)

#Converting the rpm to velocity
def rpm_to_velocity(rpm1,rpm2):
    v1 = (2*np.pi*Wheel_Radius*rpm1)/60
    v2 = (2*np.pi*Wheel_Radius*rpm2)/60
    return v1,v2

#Defining the bactracking algorithm 
def B_tracking(Pth, initial_pt):
    b_track = []
    
    a = list(Pth)[-1]
    K = Pth.get(a)
    b_track.append(a)
    b_track.append(K)
    
    while K != (initial_pt):  
        K = Pth.get(K)
        
        b_track.append(K)
    b_track.reverse()
    return (b_track)

def generateVelocityPath(path, rpmDict):
    velocityPath = []
    for i, key in enumerate(path):
        if i !=0:
            velocityPath.append(rpmDict[key])
    print("\nvelocityPath: ", velocityPath, "\nvelocityPath length: ", len(velocityPath), "\nvelocityPath shape: ", len(velocityPath), "x", len(velocityPath[0]))
    # print("velocithPath[7]", velocityPath[7])
    return velocityPath
         
space = np.ones((201,601,3),dtype='uint8')  #Creating an matrix with ones, of the shape of boundry shape

Obstacle_Clearance = int(input("Enter the Obstacle Clearance of the Robot: "))
obstacle_space(space)           #Creating the obstacle boundries

Obs_Coords= boundry_creation(space)

# for val in Obs_Coords:
#     if val == (101,70):
#         print("the val is present in obstacle", val)

initial_pt = User_Inputs_Start(Obs_Coords)  
goal_pt = User_Inputs_Goal(Obs_Coords)
rpm_1,rpm_2 = User_Input_rpm()
vel_1,vel_2 = rpm_to_velocity(rpm_1,rpm_2)
#Length_of_stepsize = int(input("Enter the stepsize of the robot in units bet(1<=stepsize<=10): "))

#print(initial_pt)
#print(goal_pt)
start = (0,0,0,initial_pt)
#print(start)
InitialEucledian_dist = np.sqrt(((goal_pt[0] - start[3][0])**2)+((goal_pt[1] - start[3][1])**2))  
InitialTotalCost = InitialEucledian_dist   
start = (InitialTotalCost,InitialEucledian_dist,0,initial_pt)
#print(start)
UncheckedList.put(start)
#print(UncheckedList)
reached=0
while UncheckedList.qsize() != 0:
    a = UncheckedList.get()
    # print(a)
    #print(goal_pt)
    if CheckedList[int(round(a[3][1])), int(round(a[3][0]))] != 1:
        CheckedList[int(round(a[3][1])), int(round(a[3][0]))] = 1
        if a[1] > 5:
            zero_n_rpm1(a)     
            rpm1_n_zero(a)
            rpm1_n_rpm1(a)
            zero_n_rpm2(a)
            rpm2_n_zero(a)
            rpm2_n_rpm2(a)
            rpm1_n_rpm2(a)
            rpm2_n_rpm1(a)
            # for i in CloseList:
            #     j = Pth.get(i)
            #     cv.line(space,(i[0],250-i[1]),(j[0],250-j[1]),(0,0,255),1)
                
            #     cv.imshow("Space",space)
            #     if cv.waitKey(50) & 0xFF == ord('q'):
            #         break

        else:
            print("success")
            reached=1
            break
if reached ==1:
    b = B_tracking(Pth, initial_pt)
    print("path: ", b, "path length: ", len(b))
    print("\nrpmDict: ", rpmDict)
    velPath = generateVelocityPath(b, rpmDict)
    
    for i in CloseList:
        xi,yi,teta = i[0],i[1],round(i[2])
        j = Pth.get((xi,yi,teta))
        cv.line(space,(int(i[0]),200-int(i[1])),(int(j[0]),200-int(j[1])),(0,0,255),1)
        
        # cv.imshow("Space",space)
        ScaledUp = cv.resize(space, (1202, 402))
        cv.imshow("ScaledUp", ScaledUp)
        if cv.waitKey(20) & 0xFF == ord('q'):
            break

    for j in b:
        space[200-int(j[1])][int(j[0])] = [0,255,0]
        cv.circle(space,(int(j[0]),200-int(j[1])), Robot_Radius, (0,255,0), -1)
        # cv.imshow("SPACE", space )
        SPACEscaledUp = cv.resize(space, (1202, 402))
        cv.imshow("SPACE", SPACEscaledUp)
        if cv.waitKey(50) & 0xFF == ord('q'):
            break


else:
    print("The Goal node cannot be reached")
# plt.grid()
# ax.set_aspect('equal')
# plt.xlim(0,600)
# plt.ylim(0,200)
# plt.title('How to plot a vector in matplotlib ?',fontsize=10)
# plt.show()
# plt.close()

# cv.imshow("SPACE", space )
# cv.waitKey()

cv.destroyAllWindows()


# velocityList = [[(1, 2, 0), (2, 5, 0), (5, 5, 0), (5, 5, 0), (5, 5, 0), (5, 2, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)], \
#                 [(2, 2, 0), (2, 5, 0), (5, 5, 0), (5, 5, 0), (5, 5, 0), (5, 2, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)], \
#                 [(3, 5, 0), (2, 5, 0), (5, 5, 0), (5, 5, 0), (5, 5, 0), (5, 2, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)], \
#                 [(4, 5, 0), (2, 5, 0), (5, 5, 0), (5, 5, 0), (5, 5, 0), (5, 2, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)]]

# [rpm1, rpm2]
# velocityList = [[(2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0), (2, 5, 0)]]

# [0, rpm2]
# velocityList = [[(0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0), (0, 5, 0)]]

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

                    # print("t = ", t, "on iteration: ", i)
                    # print()
                            # (m / m) * (rpm) * (2pi rad/1rpm) * (1min/60s) * s
                    thetaN += (r / L) * (UR - UL) * 2*pi / 60 * dt # rad
                    
                                    # (m / m) * (rpm - rpm) * (1rad/1rpm) * cos
                    # msg.linear.x = (r / 2) * (UL + UR) * 2*pi * cos(thetaN) # m/s check units
                    # msg.linear.y = (r / 2) * (UL + UR) * 2*pi * sin(thetaN)
                    Vx = (r / 2) * (UL + UR) * 2*pi/60 * cos(thetaN) # m/s check units
                    Vy = (r / 2) * (UL + UR) * 2*pi/60 * sin(thetaN)
                    # if UL == UR:
                    # print("UL == UR")
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
        publishVelocities(velPath)
    except rospy.ROSInternalException:
        pass