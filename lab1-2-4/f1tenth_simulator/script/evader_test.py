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

def receive(msg):

    #for in scan_ranges == rangse[-45, 45] in ranges:
        # if msg.ranges
    # scan_ranges_right = msg.ranges[180:270]
    # scan_ranges_left = msg.ranges[270:360]
    # # avg = sum(scan_ranges)/len(scan_ranges)
    # min_ranges_right = min(scan_ranges_right)
    # min_ranges_left = min(scan_ranges_left)
    # scan_ranges= msg.ranges[180:360]
    # min_scan = min(scan_ranges)
    
    # full detect area

    front_ranges = msg.ranges[180-15:360+15]
    if min(front_ranges) < 5:
        detect = msg.ranges[0:539]
        threshold = 0.7
        space_direction = getSpace(detect, threshold)
        # rand_index =  random.randint(0,len(space_direction)-1)
        if space_direction != [-270]:
            direction = random.sample(space_direction, 1)[0]
            if direction < 270:
                #Turn right
                angle = direction - 270
                print("TURNING RIGHT")
            else:
                angle = direction - 270
                print("TURNING LEFT")
                
            print(angle)
            drive_msg.drive.steering_angle = angle
            drive_msg.drive.speed = 1
            drive_msg.drive.steering_angle_velocity = 0
            drive_pub.publish(drive_msg)
            # rate = rospy.Rate(10)
            # rate.sleep()
        else:
            drive_msg.drive.steering_angle = -3.14/4
            drive_msg.drive.speed = -2
            drive_msg.drive.steering_angle_velocity = 2
            drive_pub.publish(drive_msg)
            drive_msg.drive.steering_angle = 0
            print("BACKKKKKKKKKKKKKKKKKKKKKK")
            rate = rospy.Rate(800)
            rate.sleep()
            
    
    
    
    
    # if min_scan < 2:
    #     rand_right = random.uniform(3.14, 3.14*2)
    #     drive_msg.drive.steering_angle = rand_right
    #     drive_msg.drive.speed = 0.1
    #     drive_msg.drive.steering_angle_velocity = 1
    #     scan_ranges_right = msg.ranges[180:270]
    #     # min_ranges_right = min(scan_ranges_right)
    #     drive_pub.publish(drive_msg)
    #     print(rand_right)

    #     while msg.ranges[270] < 2:

    #         print("HERE")
           
    #     print("OUT")
    #     #if min_ranges > 5:
            
            
    #     # print(rand)
    #     # steering_angle
    #     # drive_msg.drive = move
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
    


            
            






            
            


             




        
