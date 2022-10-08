#!/usr/bin/env python
#reference:wiki.ros.org/tf/Tutorials  
import roslib
roslib.load_manifest('f1tenth_simulator')
import rospy
import math
import tf
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
drive_msg = AckermannDriveStamped()


if __name__ == '__main__':
   rospy.init_node('robot_1')
   listener = tf.TransformListener()

   turtle_vel = rospy.Publisher('/evader_drive', AckermannDriveStamped, queue_size=1)

   rate = rospy.Rate(10.0)
   while not rospy.is_shutdown():
       try:
           (trans,rot) = listener.lookupTransform('/robot_1', '/robot_0', rospy.Time(0))
       except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
           continue

       angular = 4 * math.atan2(trans[1], trans[0])
       linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

       drive_msg.drive.speed = linear
       drive_msg.drive.steering_angle = angular
       turtle_vel.publish(drive_msg)

       rate.sleep()


