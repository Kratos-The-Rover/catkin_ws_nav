#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import math
import tf
import time

MAX_ANGULAR_VEL = 2
current_yaw = 0
class husky_controller():
	def __init__(self):
		self.pose = Twist()
		self.odom = Odometry()
		self.flag=0
		self.init_twist()


		self.vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)
		self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_cb)
		rate=rospy.Rate(100)

	def odom_cb(self,data):
		#time1=time.time()
		self.pose.linear.x = data.pose.pose.position.x
		self.pose.linear.y = data.pose.pose.position.y
		self.pose.linear.z = data.pose.pose.position.z
		# print data.pose.pose.position.x
		# print data.pose.pose.position.y
		print "x   ",abs(self.pose.linear.x-dest_x)
		print "y   ",abs(self.pose.linear.y-dest_y)
		quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
		euler=tf.transformations.euler_from_quaternion(quaternion)

		self.pose.angular.x = euler[0]
		self.pose.angular.y = euler[1]
		self.pose.angular.z = euler[2]
		self.last_yaw=current_yaw
		current_yaw=self.pose.angular.z
		

		
		# print "current_yaw", current_yaw

		if((abs(self.pose.linear.x-dest_x)<0.5) and (abs(self.pose.linear.y-dest_y)<0.5)):
			print "condition satisfied"
			twist_obj=Twist()
			twist_obj.linear.x=0
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=0
			self.flag=0
			self.vel_pub.publish(twist_obj)

		if((abs(current_yaw-dest_yaw)>0.01)):
			twist_obj=Twist()
			twist_obj.linear.x=0
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=(kp*(dest_yaw-current_yaw)* MAX_ANGULAR_VEL + kd*(current_yaw-self.last_yaw))
			print twist_obj.angular.z
			self.vel_pub.publish(twist_obj)
		if((abs(current_yaw-dest_yaw)<0.01)):
			self.flag=1
		if((abs(self.pose.linear.x-dest_x)>0.5) and (abs(self.pose.linear.y-dest_y)>0.5) and self.flag==1):
			twist_obj=Twist()
			twist_obj.linear.x=fwd_vel
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=0
			print twist_obj.angular.z
			self.vel_pub.publish(twist_obj)

		#duration=time.time()-time1
	def init_twist(self):
		self.pose.linear.x = 0
		self.pose.linear.y = 0
		self.pose.linear.z = 0
		self.pose.angular.x = 0
		self.pose.angular.y = 0
		self.pose.angular.z = 0

if __name__=="__main__":
	

	fwd_vel=0.5
	kp = 0.5
	kd = 4000000
	dest_x = 5
	dest_y = 5
	dest_yaw = math.asin(dest_y/(math.sqrt(dest_x**2+dest_y**2)))

	rospy.init_node("husky_controller")
	my_controller = husky_controller()
	rospy.spin()

