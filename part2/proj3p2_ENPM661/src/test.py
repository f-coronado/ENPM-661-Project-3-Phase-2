#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

Robot_Radius = .105     # converted mm to m


def obstacle_space(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if ((250-l) - 5 < 0) or ((m) - 5 < 0) or ((250-l) - 245 > 0) or ((m) - 595 > 0): #boundary
                space[l][m] = [0,0,255]
                
            if (m > (100-(Obstacle_Clearance+Robot_Radius))) and (m < (151+(Obstacle_Clearance+Robot_Radius))) and (250-l < 101+(Obstacle_Clearance+Robot_Radius)) and (250-l > 2): #rectangle1 with robot radius clearance
                space[l][m] = [20,0,0]
                
            if (m > 100-Obstacle_Clearance) and (m < 151+Obstacle_Clearance) and (250-l < 101+Obstacle_Clearance) and (250-l >2):     #rectangle1 with boundry clearance
                space[l][m] = [0,255,0]
                
            if (m > 100) and (m < 151) and (250-l < 101) and (250-l > 2):   #rectangle1
                space[l][m] = [0,0,255]
                
            if(m > 100-(Obstacle_Clearance+Robot_Radius)) and (m < 151+(Obstacle_Clearance+Robot_Radius)) and (250-l >150-(Obstacle_Clearance+Robot_Radius)) and (250-l < 248):
                space[l][m] = [20,0,0]
                
            if(m > 100-Obstacle_Clearance) and (m < 151 + Obstacle_Clearance) and (250-l >150-Obstacle_Clearance) and (250-l < 248):
                space[l][m] = [0,255,0]
                
            if (m > 100) and (m < 151) and (250-l >150) and (250-l < 248):  #rectangle2
                space[l][m] = [0,0,255]
            new_v1 = 223.205 - Obstacle_Clearance*1.154
            new_v2 = 26.795 + Obstacle_Clearance*1.154
            new_v3 = 373.207 + Obstacle_Clearance*1.154
            new_v4 = 123.207 +Obstacle_Clearance*1.154    
            new_c1 = new_v1 - Robot_Radius*1.154
            new_c2 = new_v2 + Robot_Radius*1.154
            new_c3 = new_v3 + Robot_Radius*1.154
            new_c4 = new_v4 + Robot_Radius*1.154
            if (((0.577*m)+(250-l)-new_c1)>=0) and ((m-235.048+(Obstacle_Clearance+Robot_Radius))>=0) and (((-0.577*m)+(250-l)-new_c2<=0)) and (((0.577*m)+(250-l)-new_c3)<=0) and ((m-364.951-(Obstacle_Clearance+Robot_Radius))<=0) and (((-0.577*m)+(250-l)+new_c4)>=0):
                space[l][m] = [20,0,0]
                
            if (((0.577*m)+(250-l)-new_v1)>=0) and ((m-235.048+Obstacle_Clearance)>=0) and (((-0.577*m) + (250-l)-new_v2<=0)) and (((0.577*m)+(250-l)- new_v3)<=0) and ((m-364.951-Obstacle_Clearance)<=0) and (((-0.577*m)+(250-l)+new_v4)>=0):
                space[l][m] = [0,255,0]
                
            if (((9375*m)+(16238*(250-l))-3624400)>=0) and (((125*m)-29381)>=0) and (((9375*m)-(16238*(250-l))+435100)>=0) and (((37500*m)+(64951*(250-l))-24240200)<=0) and (((1000*m)-364951)<=0) and (((37500*m)-(64951*(250-l))-8002450)<=0):   #hexagon
                space[l][m] = [0,0,255]

            t_v1 = 1145 + Obstacle_Clearance*2.23
            t_v2 = 895 + Obstacle_Clearance*2.23    
            t_c1 = t_v1 + Robot_Radius*2.23
            t_c2 = t_v2 + Robot_Radius*2.23
            if(((m-460+(Obstacle_Clearance+Robot_Radius))>=0)) and (((2*m)+(250-l)-t_c1)<=0) and (((2*m)-(250-l)-t_c2)<=0) and ((250-l)>25-(Obstacle_Clearance+Robot_Radius)) and ((250-l)<(225+Obstacle_Clearance+Robot_Radius)):
                space[l][m] = [20,0,0]
                
            if ((m-460+Obstacle_Clearance)>=0) and (((2*m)+(250-l)-t_v1)<=0) and (((2*m)-(250-l)-t_v2)<=0) and ((250-l)>(25-Obstacle_Clearance)) and ((250-l)<(225+Obstacle_Clearance)):
                space[l][m] = [0,255,0]
                
            if (((m-460)>=0)) and(((2*m)+(250-l)-1145)<=0)and (((2*m)-(250-l)-895)<=0): #triangle
                space[l][m] = [0,0,255]
                
    return 

Obstacle_Clearance = int(input("Enter the Obstacle Clearance of the Robot: "))


def test():
	msg=Twist()
	pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	rospy.init_node('robot_talker',anonymous=True)
	while not rospy.is_shutdown():
		msg.angular.z=0
		msg.linear.x=-0.1
		#buff='my current time is %s" %rospy.get_time()
		pub.publish(msg)
		time.sleep(0.1)
		
if __name__=='__main__':
	test()
	
