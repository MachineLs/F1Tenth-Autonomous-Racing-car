#!/usr/bin/env python


import random

import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

drive_pub = None
max_speed = None
max_steering_angle = None


def callback(data):
    drive_msg = AckermannDrive
    drive_st_msg = AckermannDriveStamped
    drive_msg.speed = max_speed / 2.0

    rand = random.random()
    range = max_steering_angle / 2.0
    rand_ang = range * rand / 2.0

    drive_msg.steering_angle = min(max(rand_ang, -max_steering_angle), max_steering_angle)
    drive_st_msg.drive = drive_msg

    drive_pub.publish(drive_st_msg)





    rospy.init_node('random_walker')

    drive_topic = rospy.get_param("rand_drive_topic")        
    odom_topic = rospy.get_param("odom_topic")

    max_speed = rospy.get_param("max_speed")
    max_steering_angle = rospy.get_param("max_steering_angle")
    drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    rospy.Subscriber(odom_topic, String, callback)

    rospy.spin()

if __name__ == "__main__":
    main()













sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/evader_drive', AckermannDriveStamped, queue_size=10)


rospy.init_node('evader')
drive_topic = rospy.get_param('evader_drive_topic')
scan_topic = rospy.get_param('scan_topic')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/evader_drive', AckermannDriveStamped, queue_size=10)


ranges = []
drive_msg = AckermannDrive()
drive_stp_msg = AckermannDriveStamped()
laser = Laser_scan()
def callback(msg):
    global ranges
    ranges = msg.ranges



def main():
    if ranges > 1


