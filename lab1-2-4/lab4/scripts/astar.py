#!/usr/bin/env python
import math
import rospy
from re import S
import numpy as np
import heapq
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


map = np.array([
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]])

def distance(p1, p2):
      p1_x = p1[0]
      p1_y = p1[1]
      p2_x = p2[0]
      p2_y = p2[1]
      euclideanDistance = np.sqrt(((p2_x - p1_x)**2) + ((p2_y - p1_y)**2))
      return round(euclideanDistance)

# goalx goaly value is set here, and its an global variable, (I also set the fixed method in the main function, you can ignore it, just change parameter x and y here)
# goalx and goaly are the goal position in the map graph (matrix index), and the robot will move to this goal position in the map graph


def coord_change_to_m(x, y):
    c = x + 8
    r = 10 -y   
    return (c, r)


def astar(array):
    # implement A* algorithm 
    global start, goal, goalx, goaly, goalx_t, goaly_t
    start = (1,12)
    
    # goalx = 1
    # goaly = 13
    # # startx = -8
    # # starty = -2
    # goal = (goaly,goalx)
    goalx=rospy.get_param("/goalx")
    goaly=rospy.get_param("/goaly")
    # goalx= 4.5
    # goaly=9.0
    goalx = math.ceil(goalx)
    goaly = math.ceil(goaly)
    goal = coord_change_to_m(goalx, goaly)
    goaly_t, goalx_t = goal
    openset = []
    closeset = []
    temp = {}
    g_score = {start:0}
    f_score = {start:distance(start, goal)}
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    heapq.heappush(openset, (f_score[start], start))

    while openset:
        # Look for the lowest F cost square on the open list.
        current = heapq.heappop(openset)[1]
        # print('open_set',open_set)
        # print('came_from',came_from)
        if current == goal:
            path = []
            while current in temp:
                path.append(current)
                current = temp[current]
            path.reverse()
            return path
        
        # add it to the closed list
        closeset.append(current)
        
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            G_score_new = g_score[current] + distance(current, neighbor)
            
            if 0 <= neighbor[0] < array.shape[1]:
                # print("neighbor[0]",neighbor[0])
                if 0 <= neighbor[1] < array.shape[0]:
                    # print("neighbor[1]",neighbor[1])
                    if array[neighbor[1]][neighbor[0]] == 1:
                        continue
                else:
                    continue
            else:
                continue
            
            # If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following.
            judge = G_score_new >= g_score.get(neighbor, 0)

            if neighbor in closeset and judge:
                continue
            
            judge_2 = [i[1]for i in openset]

            if  G_score_new < g_score.get(neighbor, 0) or neighbor not in judge_2:
                temp[neighbor] = current
                g_score[neighbor] = G_score_new
                f_score[neighbor] = G_score_new + distance(neighbor, goal)
                heapq.heappush(openset, (f_score[neighbor], neighbor))

    raise ValueError("No Path Found")

def robot_move(data,judege_state):
    if True:
        msg = Twist()
        path
        nextNode=judege_state['nextNode']
        pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        global currentPosition, rot, rotbotW
        currentPosition=data.pose.pose.position
        currentOrientation=data.pose.pose.orientation
        current_angle=euler_from_quaternion([currentOrientation.x,currentOrientation.y,currentOrientation.z,currentOrientation.w])[2]
        # print(robotW)
        robotposx=currentPosition.x
        robotposy=currentPosition.y
        # avoid collision, there is an wall in the best
        goal_x=path[nextNode][0]-8.3
        goal_y=9.2-path[nextNode][1]
        theta=np.arctan2(goal_y-robotposy,goal_x-robotposx)
        bias = theta - current_angle
        P = np.pi
        # based on the bias, the robot should turn left or right or go straight
        if judege_state['judge2']:
            if np.abs(bias) > np.radians(5):
    
                if bias < 0:
                    bias += P * 2
                    
                elif bias > P * 2:
                    bias -= P * 2
                    
                if bias > P:
                    msg.angular.z = -1
                    pub.publish(msg)
                else:
                    msg.angular.z = 1
                    pub.publish(msg)
            else:
                judege_state["judge2"]=False
        else:
            bias=np.sqrt((goal_x-robotposx)**2+(goal_y-robotposy)**2)
            if bias> 0.5:
                msg.linear.x = 0.75
                pub.publish(msg)
            else:
                # arrived
                judege_state["judge2"]=1
                if nextNode+1<len(path):
                    judege_state["nextNode"]=nextNode+1
                    # judege_state['nextNode']+=1
                else:
                    judege_state['judgeGoal']=2

def main():
    judge_state={}
    judge_state['judgeGoal']=1
    judge_state["judge2"]=1
    judge_state['nextNode']=0
    global path
    path=astar(map)
    # fix the path
    goal_x = goalx_t-1
    goal_y = goaly_t
    end_param=[goal_y,goal_x]
    path.append(end_param)
    # print(path)
    rospy.init_node("robot", anonymous=False)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, robot_move, judge_state)                              
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




