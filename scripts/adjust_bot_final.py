#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from tf import TransformListener

MAX_ANGULAR_VEL = 0.0005

class Follower():

  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('darknet_ros/bounding_boxes',BoundingBoxes, self.image_callback)
    self.object_cnt_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.object_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=5)
    
    self.twist = Twist()
    print("follower class initialised")
    self.rate = rospy.Rate(10)    


  def image_callback(self, msg):
    #take cx and cy from topic.Consider msg=[x_min,y_min,x_max,y_max]
    self.valuexmin=msg.bounding_boxes[0].xmin 
    self.valuexmax=msg.bounding_boxes[0].xmax
    self.valueymin=msg.bounding_boxes[0].ymin    
    self.valueymax=msg.bounding_boxes[0].ymax
    #print(self.valuexmin)
    #print(msg.bounding_boxes)
    
    if msg.bounding_boxes[0].Class == "sportsball":

        print("Detected Person")

        self.cx = (self.valuexmin + self.valuexmax)/2
        self.cy = (self.valueymin + self.valueymax)/2
        #cv2.circle(Image, (self.cx, self.cy), 20, (0,0,255), -1)
        #self.h, self.w, self.d = Image.shape
        self.err = 640-self.cx
	print("Error is: ",self.err)
        self.twist.linear.x = 0
	self.twist.angular.z = 0
	if((self.err)>250 or (self.err)<-250):
	        self.twist.angular.z = float(self.err) / 500
		if self.twist.angular.z > MAX_ANGULAR_VEL:
                	self.twist.angular.z = MAX_ANGULAR_VEL
		if self.twist.angular.z < -MAX_ANGULAR_VEL:
			self.twist.angular.z = -MAX_ANGULAR_VEL
	else:
		self.twist.angular.z = 0
		self.twist.linear.x = 0.001
        self.cmd_vel_pub.publish(self.twist)
	
        #cv2.imshow('target',Image)
        #cv2.waitKey(3)

    	print("Published Velocity", self.twist)
    
    if "person" not in msg.bounding_boxes[0].Class:
	self.twist = Twist()
	for i in range(10):
	  self.twist = Twist()        
	  self.cmd_vel_pub.publish(self.twist)

    
  def object_callback(self, msg):
    self.object_cnt = msg.count
    if self.object_cnt == 0:
      for i in range(10):
	self.twist = Twist()        
	self.cmd_vel_pub.publish(self.twist)
    
    self.rate.sleep()
	
if __name__ == "__main__":
  try:
    rospy.init_node('follower')
    follower = Follower()

    rospy.spin()
  except exception as e:
    print("exception is: ", e)
