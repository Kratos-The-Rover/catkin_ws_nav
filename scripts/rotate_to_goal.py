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
 
roll = pitch = yaw = 0.0
x = y = z = 0
#target = 90
kp=0.5
flag=1
 
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

def execute(goal):
    print("in execute")
    global x,y,yaw,command,kp,server,r,flag,pub
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
        target_rad = dest_yaw=math.atan2(y_diff,x_diff)
        if ((target_rad-yaw)>0.01):
            command.angular.z = kp * (target_rad-yaw)
            feedback = RotateToGoalFeedback()
            feedback.angle_left = abs(target_rad-yaw)
            server.publish_feedback(feedback)
            pub.publish(command)
        else:
            command.angular.z=0
            pub.publish(command)
            result = RotateToGoalResult()
            result.result = True
            server.set_succeeded(result, "Goal reached successfully")
            flag=0
        print("publishing")
        print("target_angle={} current_angle:{}", target_rad,yaw)
        
if __name__=="__main__":
    rospy.init_node('rotate_robot')
    command =Twist()
    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    print("after subscriber")
    pub =  rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size=10)
    server = actionlib.SimpleActionServer('rotator', RotateToGoalAction, execute, False)
    server.start()
    r = rospy.Rate(1)
    
    # while not rospy.is_shutdown():

    #     r.sleep()
    rospy.spin()

