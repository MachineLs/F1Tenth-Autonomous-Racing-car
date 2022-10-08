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

# The author is Lunshu Sun
# the initial speed is set to 2 in 64 line and the control key is E, Thank you!
# Main method is to find a black area to move forward and also add a back car funtion to avoid collision...

# college value and set a threshold to judge the action

def getSpace(ran, thresh):
    num = len(ran)
    space_direct = []
    for i in range(num):
        if ran[i] > thresh:
            space_direct.append(i)
    
    if space_direct == []:
        space_direct =[-270]
    
    return space_direct

def callback(msg):
    # full detect area and compare to the threhold to decide a random blank area to go forward
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
                
            	
            drive_msg.drive.steering_angle = angle
            drive_msg.drive.speed = 1
            drive_msg.drive.steering_angle_velocity = 0
            drive_pub.publish(drive_msg)
            # rate = rospy.Rate(10)
            # rate.sleep()
        else:
	    # back car function and the speed is set to -2	
            drive_msg.drive.steering_angle = -3.14/4
            drive_msg.drive.speed = -2
            drive_msg.drive.steering_angle_velocity = 2
            drive_pub.publish(drive_msg)
            drive_msg.drive.steering_angle = 0
            print("BACKKKKKKKKKKKKKKKKKKKKKK")
            rate = rospy.Rate(800)
            rate.sleep()
    else:
        drive_msg.drive.speed = 2.0
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.steering_angle_velocity = 0.0
        drive_pub.publish(drive_msg)

rospy.init_node('evader')



drive_sub = rospy.Subscriber('/scan', LaserScan, callback)
drive_pub = rospy.Publisher('/evader_drive', AckermannDriveStamped, queue_size=1)
rospy.spin()      

    


            
            






            
            


             




        
