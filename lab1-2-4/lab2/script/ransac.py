#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import numpy as np

# Frame Rate set to 5
# The Green line is ransac_line and the Red line is the actual_line
# the loop for each epoch is updated by P accuracy
# set an min_max value here for loop here to avoid large computing and delay

def ransac():
    global ranges
    rate = rospy.Rate(10)
    pub1 = rospy.Publisher("ransac_vis", Marker, queue_size=10)
    pub2 = rospy.Publisher('actual_vis', Marker, queue_size=10)

    while not rospy.is_shutdown():
        try:
            global draw_x_points, draw_y_points
            loop = 100
            Epoch = 10
            x = ranges[:,0]
            y = ranges[:,1]
            ln = len(x)
            theIndex = range(ln)
            draw_x_points = [-1]
            draw_y_points = [-1]
            threshold = ln * 0.001
            P = 0.99
            print(loop)
            
            for _ in range(Epoch):
                inliers = []
                outliers = []

                for i in range(loop):
                    temp_inliers = []
                    temp_outliers = []
                    # random select two points
                    idx1 = np.random.randint(0,len(theIndex)-1)
                    idx2 = np.random.randint(0,len(theIndex)-1)
                    # sample = random.sample(theIndex,2)
                    # idx1 = int(sample[0])
                    # idx2 = int(sample[1])
                    if idx1 == idx2:
                        continue
                    x1 = x[theIndex[idx1]]
                    y1 = y[theIndex[idx1]]
                    
                    x2 = x[theIndex[idx2]]
                    y2 = y[theIndex[idx2]]
                    
                    if x1 != 0 and x2 != 0:
                        # select a point (x0,y0) to judge if its distance to the line is samller than threshold
                        for j in theIndex:
                            x0 = x[j]
                            y0 = y[j]
                            try :
                                # calculate the distance       
                                distance = abs(((y2-y1)*x0) - ((x2-x1)*y0) + ((x2*y1) - (y2*x1))) / math.sqrt(((y2-y1)**2) + ((x2-x1)**2))
                                        
                            except ZeroDivisionError:
                                        
                                continue
                            
                            if distance < threshold:
                                temp_inliers.append(j)
                            elif distance > threshold:
                                temp_outliers.append(j)
                    # print(len(temp_inliers))
                    
                    if len(inliers) < len(temp_inliers):
                            #print(temp_inliers)
                            try :
                                # calculate the distance       
                                upper = np.log(1 - P)
                                #print(upper)
                                a = int(len(temp_inliers))               
                                b = 361                             
                                # update my loop use the ransac fomula, however my w=a/b is always 0
                                # (even if the a is 200+ and b is 361, maybe some bug in old version python or ros)
                                # an 1.0 shoule be multiply so I can get the correct value.
                                w = a*1.0/b
                                # # w = np.divide(len(temp_inliers), 361)
                                # print('w is a/b', w)
                                
                                # set a tolerance here, if lower is near 0, set it to a bigger value than 0
                                wn = w**len(temp_inliers)
                                if wn < 10**(-5):
                                    wn = 10**(-2)
                                # print(wn)
                                lower = np.log(1-wn)
                                
                                # print(lower)
                                loop = round(upper/lower)

                                # set an min max value for loop, or the ransac line will have a very big delay and is hard to see in rviz
                                # to acieve 0.99 the value loop will change to thousands and hard to compute
                                if loop < 50 :
                                    loop = 50
                                elif loop > 200:
                                    loop = 200
                                    
                            except ZeroDivisionError:
                                        
                                continue
                        
                            inliers = temp_inliers
                            outliers = temp_outliers
                            # max_x, min_x, max_y, min_y = maxValueHelper(inliers)
                            # print(max_x, min_x, max_y, min_y)
                            max_x = np.max(ranges[inliers,0]) 
                            min_x = np.min(ranges[inliers,0])
                            end_point = ranges[inliers,:]
                            max_y = end_point[np.argmax(end_point[:,0]), 1]
                            min_y = end_point[np.argmin(end_point[:,0]), 1]
                            
                    print(loop)
                
                # The number of nearby points requireed - at least 4 points    
                if len(inliers) > 10:
                    maxValueHelper(max_x, min_x, max_y, min_y)
                    theIndex = outliers
                
            if len(inliers) > len(theIndex)//2:
                continue
            
            '''  Set ransac line '''    
            
            marker_ransac = Marker()
            marker_ransac.header.frame_id = "/base_link"
            marker_ransac.header.stamp = rospy.Time.now()
            marker_ransac.ns = "ransac_line"   
            marker_ransac.id = 0       
            marker_ransac.type = Marker.LINE_LIST
            marker_ransac.action = Marker.ADD
            marker_ransac.lifetime = rospy.Duration()            
            # set ransac_line color to Green
            marker_ransac.scale.x = 0.1
            marker_ransac.color.g = 1.0
            marker_ransac.color.a = 1.0  
            
            '''  Set actual line '''
            
            marker_actual = Marker()
            marker_actual.header.frame_id = "/base_link"
            marker_actual.header.stamp = rospy.Time.now()
            marker_actual.ns = "actual_line"
            marker_actual.id = 1
            marker_actual.type = Marker.LINE_STRIP
            marker_actual.action = Marker.ADD
            # set ransac_line color to Blue
            marker_actual.scale.x = 0.1
            marker_actual.color.r = 1.0
            marker_actual.color.a = 1.0
            marker_actual.lifetime = rospy.Duration()
            
            '''  Draw ransac line '''
            for i in range(1,len(draw_x_points)):
                p_sansac = Point()
                p_sansac.x = draw_x_points[i]
                p_sansac.y = draw_y_points[i]
                marker_ransac.points.append(p_sansac)

            '''  Draw actual line '''
            for i in range(361):
                if ranges[i,0] == 0 and ranges[i,1] == 0:
                    continue
                p_actual = Point()
                p_actual.x = ranges[i,0]
                p_actual.y = ranges[i,1]
                marker_actual.points.append(p_actual)
                
            # Publish both marker
            pub1.publish(marker_ransac)
            pub2.publish(marker_actual)
        
        except:
            
            continue
        
def callback(msg):
    global ranges
    ranges = np.zeros((361,2))
    # split 360 degrees from a circle
    degrees = np.linspace(np.pi/2,-1 * np.pi / 2 , 361)
    # convert angular theta and range R to (x,y) in the map
    sin_x = np.sin(degrees)
    cos_x = np.cos(degrees)
    theRange = np.array(msg.ranges)
    # set r to 0 because the range_max is 3.0
    # or the rviz will show a a circle_line with max value R

    theRange[theRange == 3.0] = 0
    y = sin_x * theRange
    x = cos_x* theRange
    # print(x, y)
    # apply converted (x,y) to replace ranges
    ranges[:,0:1] = x.reshape((x.shape[0], 1))
    ranges[:,1:] = y.reshape((y.shape[0], 1))
        
        		        
def maxValueHelper(max_x, min_x, max_y, min_y):
    
    draw_x_points.append(max_x)
    draw_y_points.append(max_y)
    draw_x_points.append(min_x)
    draw_y_points.append(min_y)
    return draw_x_points, draw_y_points
    
if __name__ == '__main__':
    try:
        rospy.init_node('ransac', anonymous=True)
        rospy.Subscriber("/ransac/base_scan", LaserScan, callback)
        ransac()
    except rospy.ROSInterruptException:
        pass
