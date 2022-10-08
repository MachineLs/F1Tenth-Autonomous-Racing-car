#!/usr/bin/env python


import rospy
import random
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):

    if msg.ranges[0] > 1 and msg.ranges[90] >1 and msg.ranges[270] >1 :
      move.linear.x = 2.0
      move.angular.z = 0.0
    else:
      turn_angle = random.randrange(0,360)
      move.linear.x = 0.0
      move.angular.z = turn_angle
      #print('mark HERE')
      #print(msg.ranges[0])
      #print('the speed is', move.linear.x)
      #print(turn_angle)  

    pub.publish(move)

rospy.init_node('evader')
sub = rospy.Subscriber('/base_scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move = Twist()

rospy.spin()

