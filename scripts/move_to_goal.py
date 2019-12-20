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
 
roll = pitch = yaw = 0.0
x = y = z = 0
#target = 90
fwd_vel= 0.3
flag=1
 
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
        if ((x_diff>0.1) or (y_diff>0.1)):
            command.linear.x = fwd_vel
            feedback = moveToGoalFeedback()
            feedback.distance_left = math.sqrt(x_diff**2+y_diff**2)
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

