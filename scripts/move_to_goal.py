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
 
 
#Initializing global variables
roll = pitch = yaw = 0.0
x = y = z = 0
#target = 90
fwd_vel= 0.3
flag=1

'''
Setting the current position of the bot
'''
 
def get_position (msg):
    global roll, pitch, yaw, x, y, z
    #orientation_q = msg.pose.pose.orientation
    position_q= msg.pose.pose.position
    x=position_q.x
    y=position_q.y
    z=position_q.z
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
    print("in execute")
    global x,y,command,fwd_vel,server,r,flag,pub
    flag=1
    while flag:
        #quat = quaternion_from_euler (roll, pitch,yaw)
        #print quat
        
        print("in while loop")
        p = goal.goal.point
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
        if ((abs(x_diff)>0.5) or (abs(y_diff)>0.5)):
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
        print("publishing")
        print("target pos={} current pos:{}", (dest_x,dest_y),(x,y))
        
        
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

