#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
import actionlib
from navigation.msg import RotateToAngleGoal, RotateToAngleAction, Point_xy, PointArray


MAX_ANGULAR_VEL = 0.0005

class Follower():

  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('darknet_ros/bounding_boxes',BoundingBoxes, self.image_callback)
    self.object_cnt_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.object_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=5)
    self.twist = Twist()
    self.rotate_client = actionlib.SimpleActionClient('rotator',RotateToAngleAction)
    self.rotate_client.wait_for_server()
    print("follower class initialised")
    self.rate = rospy.Rate(10)    

  def image_callback(self, msg):
    
    #print(self.valuexmin)
    #print(msg.bounding_boxes)
    if self.object_cnt!=0:
    
        if msg.bounding_boxes[0].Class == "right" or msg.bounding_boxes[0].Class =="left":   #check index number for left and right class

            #take cx and cy from topic.Consider msg=[x_min,y_min,x_max,y_max]
            self.xmin=msg.bounding_boxes[0].xmin  
            self.xmax=msg.bounding_boxes[0].xmax
            self.ymin=msg.bounding_boxes[0].ymin    
            self.ymax=msg.bounding_boxes[0].ymax

            print("Detected an arrow")
            print("msg.bounding_boxes[0].Class")

            self.cx = (self.xmin + self.xmax)/2
            self.cy = (self.ymin + self.ymax)/2
            #cv2.circle(Image, (self.cx, self.cy), 20, (0,0,255), -1)
            #self.h, self.w, self.d = Image.shape

            ### Converting ROS Image into cv_image
            bridge = cv_bridge.CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data)

            ### Depths of corners and center of bounding box

        #	print(cv_image[xmin][ymax])
        #	print(cv_image[xmax][ymax])
        #	print(cv_image[cx][cy])

            self.err = 640-self.cx
            print("Error is: ",self.err)
            self.twist.linear.x = 0
            self.twist.angular.z = 0

        #    avg_depth = (cv_image[cx][cy]+cv_image[self.xmax][self.ymax]+cv_image[self.xmax][self.ymin]+cv_image[self.xmin][self.ymax]+cv_image[self.xmin][self.ymin])/5.0
            avg_depth = cv_image[cx][cy]

            if(avg_depth>2):
                if((self.err)>250 or (self.err)<-250):
                    self.twist.angular.z = float(self.err) / 500
                    if self.twist.angular.z > MAX_ANGULAR_VEL:
                        self.twist.angular.z = MAX_ANGULAR_VEL
                    if self.twist.angular.z < -MAX_ANGULAR_VEL:
                        self.twist.angular.z = -MAX_ANGULAR_VEL
                else:
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0.1
                    self.cmd_vel_pub.publish(self.twist)
            else:
                d1=cv_image[self.xmin][self.ymin]
                d_mid = cv_image[self.cx][self.ymin]
                d2=cv_image[self.xmax][self.ymin]



                if(msg.bounding_boxes[0].Class=="right"):
                    if(d1>d2):
                        angle_arrow = -1*(3.142 - atan((self.cx-self.xmin)/(sqrt(d1**2 - (self.cx-self.xmin)**2)-d_mid)))
                    else:
                        angle_arrow = -1*(atan((self.max-self.cx)/(sqrt(d2**2 - (self.xmax-self.cx)**2)-d_mid)))
                if(msg.bounding_boxes[0].Class=="left"):
                    if(d2>d1):
                        angle_arrow = 3.142 - atan((self.cx-self.xmin)/(sqrt(d2**2 - (self.cx-self.xmin)**2)-d_mid)) 
                    else:
                        angle_arrow = atan((self.xmax-self.cx)/(sqrt(d1**2 - (self.xmax-self.cx)**2)-d_mid)) 
                goal2 = RotateToAngleGoal()
                goal2.goal_angle=angle_arrow
                self.rotate_client.send_goal(goal2, feedback_cb = self.rotator_fb)
                self.rotate_client.wait_for_result()
                print("Done rotating the bot")


               print("Published Velocity", self.twist)
        else:
            self.twist.angular.z = 0
            self.twist.linear.x = 0.1
            self.cmd_vel_pub.publish(self.twist)
            
    else:
        self.twist.angular.z = 0
        self.twist.linear.x = 0.1
        self.cmd_vel_pub.publish(self.twist)

    self.rate.sleep()

  def object_callback(self, msg):
    self.object_cnt = msg.count
    self.rate.sleep()
	
if __name__ == "__main__":
  try:
    rospy.init_node('follower')
    follower = Follower()

    rospy.spin()
  except exception as e:
    print("exception is: ", e)
