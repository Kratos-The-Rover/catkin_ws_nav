#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import actionlib
import roslib
roslib.load_manifest('navigation')
from navigation.msg import RotateToGoalAction, RotateToGoalGoal, RotateToGoalResult, RotateToGoalFeedback, Point_xy
 
#Initiliazing global values 
roll = pitch = yaw = 0.0
x = y = z = 0
#target = 90
kp=0.8

command=0
pub=0
server=0
 
 
'''
Setting the values of roll, pitch and yaw; x and y (current position)
Call back function of ODOM
'''
def get_rotation (msg):
    global roll, pitch, yaw, x, y, z
    orientation_q = msg.pose.pose.orientation
    position_q= msg.pose.pose.position
    x=position_q.x
    y=position_q.y
    z=position_q.z
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # print yaw
    
'''
Here we find the target angle.
'p' is the goal point
'x' and 'y'is the current position
angle is found by calculating tan inverse 
If the target_rad value is greater than the dest_rad value (actual value to which we need to rotate), we continue to rotate
otherwise we have achieved our target
'''

def execute(goal):
    print("in execute")
    global x,y,yaw,command,kp,server,r,pub
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
        target_rad = math.atan2(y_diff,x_diff)
        if (abs(target_rad-yaw)>0.005):
            print("did not reach target")
            command.angular.z = kp * (target_rad-yaw)
            feedback = RotateToGoalFeedback()
            feedback.angle_left = abs(target_rad-yaw)
            server.publish_feedback(feedback)
            pub.publish(command)
        else : 
            print("reached target angle",target_rad,yaw)
            command.angular.z=0
            pub.publish(command)
            result = RotateToGoalResult()
            result.result = True
            server.set_succeeded(result, "Goal reached successfully")
            flag=0
        print("publishing")
        print("target_angle={} current_angle:{}", target_rad,yaw)
        
'''
Initializing the Subscriber (Odom), Publisher (cmd_vel), ActionServer (Rotate_to_goal)
'''
if __name__=="__main__":
    #global pub,sub,server, command
    rospy.init_node('rotate_robot')
    command =Twist()
    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    print("after subscriber")
    pub =  rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size=10)
    server = actionlib.SimpleActionServer('rotator', RotateToGoalAction, execute, False)
    server.start()
    r = rospy.Rate(15)
    
    # while not rospy.is_shutdown():

    #     r.sleep()
    rospy.spin()

