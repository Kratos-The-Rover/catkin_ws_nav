#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from tf import TransformListener
from navigation.srv import *

MAX_ANGULAR_VEL = 0.0005
#<<<<<<< HEAD
flag=False
class Follower():

  def __init__(self):
	global flag
	self.bridge = cv_bridge.CvBridge()
	#cv2.namedWindow("window", 1)
	self.depth_sub = rospy.Subscriber('/zed/zed_node/depth/depth_registered', Image, self.detect_callback)
	self.image_sub = rospy.Subscriber('darknet_ros/bounding_boxes',BoundingBoxes, self.image_callback)
	self.object_cnt_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.object_callback)
	
#=======

class Follower():

  def __init__(self):
	self.bridge = cv_bridge.CvBridge()
	#cv2.namedWindow("window", 1)
	self.image_sub = rospy.Subscriber('darknet_ros/bounding_boxes',BoundingBoxes, self.image_callback)
	self.object_cnt_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.object_callback)
#>>>>>>> ebb477b30a378446953a3c1f3bbb31bb4523f352
	self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=5)
	
	self.twist = Twist()
	print("follower class initialised")
#<<<<<<< HEAD
	self.rate = rospy.Rate(10)
	flag=False
	self.depth_data = []    

  def detect_callback(self, data):
	bridge = cv_bridge.CvBridge()
	depth_data = bridge.imgmsg_to_cv2(data)
		
#=======
	self.rate = rospy.Rate(10)    


#>>>>>>> ebb477b30a378446953a3c1f3bbb31bb4523f352
  def image_callback(self, msg):
	#take cx and cy from topic.Consider msg=[x_min,y_min,x_max,y_max]
	self.valuexmin=msg.bounding_boxes[0].xmin 
	self.valuexmax=msg.bounding_boxes[0].xmax
	self.valueymin=msg.bounding_boxes[0].ymin    
	self.valueymax=msg.bounding_boxes[0].ymax
	#print(self.valuexmin)
	#print(msg.bounding_boxes)
	
# <<<<<#<< HEAD
	if msg.bounding_boxes[0].Class == "tennisball":

	# if msg.bounding_boxes[0].Class == "sportsball":
# >>>>>>> ebb477b30a378446953a3c1f3bbb31bb4523f352

		print("Detected Person")

		self.cx = (self.valuexmin + self.valuexmax)/2
		self.cy = (self.valueymin + self.valueymax)/2
		#cv2.circle(Image, (self.cx, self.cy), 20, (0,0,255), -1)
		#self.h, self.w, self.d = Image.shape
		self.err = 640-self.cx
		print("Error is: ",self.err)
		self.twist.linear.x = 0
		self.twist.angular.z = 0
# <<<<<<< HEAD
	if (depth_data[self.cx][self.cy] > 1):
		if((self.err)>250 or (self.err)<-250):
			self.twist.angular.z = float(self.err) / 500
			if self.twist.angular.z > MAX_ANGULAR_VEL:
						self.twist.angular.z = MAX_ANGULAR_VEL
			if self.twist.angular.z < -MAX_ANGULAR_VEL:
				self.twist.angular.z = -MAX_ANGULAR_VEL
		else:
			self.twist.angular.z = 0
			self.twist.linear.x = 0.1
	else:
		self.twist.angular.z = 0
		self.twist.linear.x = 0 ################################################ ball is in range of 1m
		self.cmd_vel_pub.publish(self.twist)
		flag=True
		#cv2.imshow('target',Image)
		#cv2.waitKey(3)
		#return AdjustResponse(True)
# =======
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

# >>>>>>> ebb477b30a378446953a3c1f3bbb31bb4523f352
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
	
# <<<<<<< HEAD
def execute(goal):
	global flag
	follower = Follower()
	while not flag:
		pass
	return AdjustResponse(True)

if __name__ == "__main__":
  try:
	rospy.init_node('follower')
	s = rospy.Service('adjust_service', Adjust, execute)
# =======  
# if __name__ == "__main__":
#   try:
# 	rospy.init_node('follower')
# 	follower = Follower()

# >>>>>>> ebb477b30a378446953a3c1f3bbb31bb4523f352
	rospy.spin()
  except Exception as e:
	print("exception is: ", e)
