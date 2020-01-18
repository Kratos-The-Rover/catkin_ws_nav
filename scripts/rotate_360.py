#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import actionlib
import roslib
roslib.load_manifest('navigation')
from navigation.msg import Rotate360Action, Rotate360Goal, Rotate360Result, Rotate360Feedback, Point_xy
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
 
roll = pitch = yaw = 0.0
x = y = z = 0
#target = 90
kp=0.8

command=0
pub=0
server=0
current_yaw = 0

def image_callback(self, msg):
    global server
    if msg.bounding_boxes[0].Class == "tennisball":
        result = Rotate360Result()
        result.result = False
        server.set_succeeded(result, "Ball detected")


 
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
    global x,y,yaw,command,kp,server,r,pub
    flag=1
    current_yaw = yaw
    while flag:

        #quat = quaternion_from_euler (roll, pitch,yaw)
        #print quat
        
        print("in while loop")
        target_rad = current_yaw-0.1
        if (abs(target_rad-yaw)>0.05):
            print("did not reach target")
            command.angular.z = 0.25
            feedback = Rotate360Feedback()
            feedback.angle_left = abs(target_rad-yaw)
            server.publish_feedback(feedback)
            pub.publish(command)
        else : 
            print("reached target angle",target_rad,yaw)
            command.angular.z=0
            pub.publish(command)
            result = Rotate360Result()
            result.result = True
            server.set_succeeded(result, "Goal reached successfully")
            flag=0
        print("publishing")
        print("target_angle={} current_angle:{} , rotating 360!!!!!!", target_rad,yaw)
        
if __name__=="__main__":
    #global pub,sub,server, command
    rospy.init_node('rotate_robot_360')
    command =Twist()
    sub = rospy.Subscriber ('zed/zed_node/odom', Odometry, get_rotation)
    image_sub = rospy.Subscriber('darknet_ros/bounding_boxes',BoundingBoxes, image_callback)
    print("after subscriber")
    pub =  rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size=10)
    server = actionlib.SimpleActionServer('rotator_360', Rotate360Action, execute, False)

    server.start()
    r = rospy.Rate(15)
    
    # while not rospy.is_shutdown():

    #     r.sleep()
    rospy.spin()

