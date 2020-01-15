#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import actionlib
import roslib
roslib.load_manifest('navigation')
from navigation.msg import moveToGoalAction, moveToGoalGoal, moveToGoalResult, moveToGoalFeedback, Point_xy
import numpy as np
 
#Initializing global variables
roll = pitch = yaw = 0.0
x = y = z = 0
#target = 90
fwd_vel= 0.3
flag=0
kl = 0.2
track_ind = 0
path=np.array([0.0,0.0])
# path = []

'''
Setting the current position of the bot
'''
 
def get_position (msg):
    global roll, pitch, yaw, x, y, z, flag
    #orientation_q = msg.pose.pose.orientation
    position_q= msg.pose.pose.position
    x=position_q.x
    y=position_q.y
    z=position_q.z
    if flag:
        get_nearest_ind()
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # print yaw
    
'''
Here we calculate the distance that needs to be traversed by the bot to send it as feedback
'p' is the current position of the bot
'dist' is the current distance that is yet to be traversed
Here we check whether the position attained by the bot is within the threshold value (with respect to the destination)
if not then we continue to move, otherwise we send the result to the client
'''

def execute(goal):
    # print("in execute")
    global x,y,command,fwd_vel,server,r,flag,pub,kl,track_ind,path
    flag=1
    start=np.array([x,y])
    # start = [x,y]
    p = goal.goal.point
    end = np.array(p)
    # end = [p[0],p[1]]
    step_size = 0.1
    dv = end - start
    # dv = [end[0]-start[0],end[1]-start[1]]
    dv = dv / np.sqrt(np.sum(dv**2))
    # dv = [dv[0] / (math.sqrt(dv[0]**2 + dv[1]**2)),dv[1] / (math.sqrt(dv[0]**2 + dv[1]**2))]
    
    path = np.array([start])
    # path.append(start)
    i=1
    # print(dv)
    while (abs(path[i-1][0])<abs(end[0]) and abs(path[i-1][1])<abs(end[1])):
        new_pt = start + i * step_size * dv
        # print(new_pt)

        path=np.concatenate((path,np.array([new_pt])),axis=0)
        print("In while : path=",path,"length=",len(path))
        # path.append([new_pt])
        i += 1

    path[-1]=end
    print(path)
    while flag:
        #quat = quaternion_from_euler (roll, pitch,yaw)
        #print quat
        # print("in while loop")
        
        dest_x = p[0]
        x_diff=dest_x-x
        dest_y = p[1]
        y_diff=dest_y-y
        dist=math.sqrt(x_diff**2+y_diff**2)

        # thres=0.5
        # if(dist>5)
        #     thres=0.7
        # if(dist>10)
        #     thres=1
        if dist>6 and not goal.flag:
            command.angular.z=0
            pub.publish(command)
            result = moveToGoalResult()
            result.result = False
            server.set_succeeded(result, "Deviated from path")
            flag=0

        if(((abs(x-path[-1][0])>0.3) or (abs(y-path[-1][1])>0.3))):
            twist_obj=Twist()
            twist_obj.linear.x=fwd_vel
            get_nearest_ind()
            twist_obj.angular.z = -kl * (path[track_ind][0] - x) 
            print("path=",path,"length=",len(path),track_ind)
            pub.publish(twist_obj)

        elif ((abs(x_diff)>0.5) or (abs(y_diff)>0.5)):
            command.linear.x = min(0.5,fwd_vel*dist)
            feedback = moveToGoalFeedback()
            feedback.distance_left = dist
            server.publish_feedback(feedback)
            pub.publish(command)
        else:
            command.angular.z=0
            pub.publish(command)
            result = moveToGoalResult()
            result.result = True
            server.set_succeeded(result, "Goal reached successfully")
            flag=0
        # print("publishing")
        # print("target pos={} current pos:{}", (dest_x,dest_y),(x,y))
        

def get_nearest_ind():
    global path,track_ind
    d = np.array([np.sqrt(np.sum(([x,y] - i)**2)) for i in path])
    track_ind = np.argmin(d)     
'''
Initializing the Subscriber(Odom), Publisher(cmd_vel), ActionServer(Move_to_goal)
'''
if __name__=="__main__":
    rospy.init_node('move_robot')
    command =Twist()
    sub = rospy.Subscriber ('/odom', Odometry, get_position)
    print("after subscriber")
    pub =  rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size=10)
    server = actionlib.SimpleActionServer('commander', moveToGoalAction, execute, False)
    server.start()
    r = rospy.Rate(1)
    
    # while not rospy.is_shutdown():

    #     r.sleep()
    rospy.spin()

