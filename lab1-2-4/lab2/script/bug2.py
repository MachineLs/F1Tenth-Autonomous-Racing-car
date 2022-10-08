#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# set initial value
star_End_Points = np.array([[-8, -2],[4.5, 9.0]])
offset = 0
Judge_front = False
Judge_left = False
JUdge_right = False
Judge_linear = False

# A is our intial point and C is our target point, B is our real-time point
# if the crossproduct of AB and AC is 0, ABC is in a line, set a threshold as 50 to help fix yaw
def go_to_point_offset(target):
    global realtime_pos
    # print(position)
    
    AB = [target[0][0] - realtime_pos.x, 
         target[0][1] - realtime_pos.y]
    
    # print(AB)
    AC = [target[0][0]- target[1][0],
          target[0][0] - target[1][1]]
    
    # print(AC)
    # AB * AC
 
    value = np.cross(AB, AC)
    print(value)
    return value < 50


def callback(msg):
    global Judge_front, Judge_left
    laser_range = msg.ranges
    front = 0
    left = 0
    right = 0
    
    laser_ranges = []
    
    # Judge if there are noises in the laser_scan value, and judge front and left
    
    for i in range(80):
        if laser_range[i] < 1:
            laser_ranges.append(laser_range[i])
            if min(laser_ranges) < 1:
                left += 1

    
    for j in range(130):
        if laser_range[130+j] < 1:
            laser_ranges.append(laser_range[130+j])
            if min(laser_ranges) < 1:
                front += 1

    for r in range(50):
        if laser_range[50] < 1:
            laser_ranges.append(laser_ranges[50])
            if min(laser_ranges) < 1:
                right += 1

    Judge_front = front > 1
    Judge_right = right > 2
    Judge_left = left > 10
     
            
def rtime_callback(msg):
    global offset
    global realtime_pos
    realtime_pos = msg.pose.pose.position
    offset = msg.pose.pose.orientation

def Judgethestate():
    global Judge_front, offset, pos, Judge_linear
    pub = rospy.Publisher("/lab2/cmd_vel", Twist, queue_size = 10)
    sub = rospy.Subscriber("/lab2/base_pose_ground_truth", Odometry, rtime_callback)
    rate = rospy.Rate(10)

    goalReached = False
    
    print(offset)


    distance_thre = 0.55
    
    following_state = "go_to_point"
    
    while not goalReached:

        if offset != 0:
            
            # Judge go to point or wallfollowing based on the angle and distance
            robot_angle = np.arcsin(offset.z)
            
            temp_distance = math.sqrt((star_End_Points[1, 0] - realtime_pos.x) ** 2 + (star_End_Points[1, 1] - realtime_pos.y) ** 2)
            
            angle_g = math.atan((star_End_Points[1, 1] - realtime_pos.y) / (star_End_Points[1,0]- realtime_pos.x)) - 2*robot_angle
            
            Judge_linear = go_to_point_offset(star_End_Points)

            velo = Twist()
            
            if temp_distance < distance_thre:
                velo.linear.x = 0 
                velo.angular.z = 0
                goalReached = True
                
                break
            
            else:
                velo.linear.x = 0.0 if Judge_front else 4.0
                
                angular_speed = 0
                
                if following_state == "Go_to_point":
                    
                    if Judge_linear:
                        
                        angular_speed = min(angle_g, 1)
                        
                        velo.linear.x = 0.5
                        
                    elif Judge_front:
                        
                        angular_speed = 1
                    else:
                        angular_speed = min(angle_g, 1)
                        
                    velo.angular.z =  angular_speed
                    
                    if Judge_front or Judge_left:
                        
                        following_state = "Wall_following"
                else:
                    
                    if Judge_front:
                        
                        angular_speed = 0.5
                        
                    else:
                        
                        if Judge_left:
                            
                            angular_speed = 0
                            
                        else:
                            
                            angular_speed = -1 * 0.8
                            
                    velo.angular.z = -1 * angular_speed
                    
                    if Judge_linear and not Judge_front:
                        
                        following_state = "Go_to_point"

            pub.publish(velo)
            rate.sleep()    
          
if __name__ == '__main__':
    try:
        
        rospy.init_node('bug2', anonymous=True)
        rospy.init_node('bug2', anonymous=True)
        rospy.Subscriber("/lab2/base_scan", LaserScan, callback)
        Judgethestate()
        
    except rospy.ROSInterruptException:
        pass
