#!/usr/bin/env python

import rospy
import random

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

move = AckermannDrive()
drive_msg = AckermannDriveStamped()

max_speed = 0
max_steering_angle = 10
scan_ranges = []

def getSpace(ran, thresh):
    num = len(ran)
    space_direct = []
    for i in range(num):
        if ran[i] > thresh:
            space_direct.append(i)
    
    if space_direct == []:
        space_direct =[-270]
    
    return space_direct

def decideLR(space_d):
    left = []
    right = []
    bigger_space = []
    for i in space_d:
        if i > 270:
            left.append(i)
        elif i == -270:
            bigger_space = [-270]
        else:
            right.append(i)
    
    if bigger_space == [-270]:
        pass
    elif len(left) >= len(right):
        bigger_space = left
    else:
        bigger_space = right

    # print("left:",left)
    # print("right:",right)
    # print(bigger_space)
    
    return bigger_space



def receive(msg):
    front_ranges = msg.ranges[180-15:360+15]
    if min(front_ranges) < 5:
        detect = msg.ranges[0:539]
        threshold = 2
        space_direction = getSpace(detect, threshold)
        # rand_index =  random.randint(0,len(space_direction)-1)
        space_direction = decideLR(space_direction)
        print(space_direction)
        if space_direction != [-270]:
            direction = random.sample(space_direction, 1)[0]
            if direction < 270:
                #Turn right
                angle = direction - 270
                # print("TURNING RIGHT")
            else:
                angle = direction - 270
                # print("TURNING LEFT")
                
            # print(angle)
            drive_msg.drive.steering_angle = angle
            drive_msg.drive.speed = 1
            drive_msg.drive.steering_angle_velocity = 0
            drive_pub.publish(drive_msg)
            # rate = rospy.Rate(10)
            # rate.sleep()
        else:
            drive_msg.drive.steering_angle = 3.14/4
            drive_msg.drive.speed = -2
            drive_msg.drive.steering_angle_velocity = 200
            drive_pub.publish(drive_msg)
            # print("BACKKKKKKKKKKKKKKKKKKKKKK")
            rate = rospy.Rate(800)
            rate.sleep()
            
    
    else:
    # #if msg.ranges[0] > 1:

        drive_msg.drive.speed = 2.0
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.steering_angle_velocity = 0.0
        drive_pub.publish(drive_msg)
        # rate = rospy.Rate(1000)
        # rate.sleep()
    # pub.Publisher(move)
    #print('*****************************')
    # print(min_ranges)
    
rospy.init_node('evader')



drive_sub = rospy.Subscriber('/scan', LaserScan, receive)
drive_pub = rospy.Publisher('/evader_drive', AckermannDriveStamped, queue_size=1)

# rospy.sleep(0.2)
rospy.spin()
    


            
            






            
            


             




        
