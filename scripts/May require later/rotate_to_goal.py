#!/usr/bin/env python
#import Jetson.GPIO as GPIO
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import math
import tf
import roslib
roslib.load_manifest('navigation')
import actionlib
from navigation.msg import RotateToGoalAction, RotateToGoalGoal, RotateToGoalResult, RotateToGoalFeedback, Point_xy
# dirs[4]={}
# pwm[4]={}

# GPIO.setmode(GPIO.BOARD)

# GPIO.setup(dirs[0], GPIO.OUT)
# GPIO.setup(dirs[1], GPIO.OUT)
# GPIO.setup(dirs[2], GPIO.OUT)
# GPIO.setup(dirs[3], GPIO.OUT)

# GPIO.setup(pwm[0], GPIO.OUT)
# GPIO.setup(pwm[1], GPIO.OUT)
# GPIO.setup(pwm[2], GPIO.OUT)
# GPIO.setup(pwm[3], GPIO.OUT)

# pwm0 = GPIO.PWM(pwm[0], 2000)
# pwm1 = GPIO.PWM(pwm[1], 2000)
# pwm2 = GPIO.PWM(pwm[2], 2000)
# pwm3 = GPIO.PWM(pwm[3], 2000)


# fwd_vel=1
# dest_x = 4
# dest_y = 4
# k1 = 8
# dest_yaw = dest_y/(math.sqrt(math.pow(dest_x,2)+math.pow(dest_y,2)))
MAX_ANGULAR_VEL = 2
flag=1


class RotateToGoalServer():

	def __init__(self):
		print("in constructor-RotateToGoalServer ")
		self.server = actionlib.SimpleActionServer('rotator', RotateToGoalAction, self.execute, False)
		self.server.start()
		# self.flag=1
		# self.current_yaw = 0
		print("Ready in RotateToGoalServer")
		rospy.spin()

	def execute(self,goal):
		while(flag):
			print("executing")
			
			self.p = goal.goal.point
			self.kp = 0.5
			self.kd = 4000000
			
			my_controller = husky_controller(self.server,self.kp,self.kd,self.p)

			print("execution finished")

class husky_controller():
	def __init__(self,server,kp,kd,p):
		print("in constructor-husky")
		self.pose = Twist()
		self.odom = Odometry()
		self.server = server
		self.dest_x = p[0]
		self.dest_y = p[1]
		self.kp = kp
		self.kd = kd
		self.dest_yaw = self.dest_y/(math.sqrt(math.pow(self.dest_x,2)+math.pow(self.dest_y,2)))
		self.init_twist()


		#to be changed
		# self.vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)
		# self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_cb)
		# rate=rospy.Rate(100)

		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size=10)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
		self.temp = 0
		self.current_yaw = 0
		rate=rospy.Rate(100)
		rate.sleep()


	def odom_cb(self,data):
		print("message received")
		
		quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
		euler=tf.transformations.euler_from_quaternion(quaternion)

		self.pose.angular.x = euler[0]
		self.pose.angular.y = euler[1]
		self.pose.angular.z = euler[2]

		#self.last_yaw=self.temp
		self.current_yaw=self.pose.angular.z
		self.temp = self.current_yaw
		print("current_yaw",self.current_yaw)
		print("dest_yaw",self.dest_yaw)
		print((abs(self.current_yaw-self.dest_yaw)))
		if((abs(self.current_yaw-self.dest_yaw)>0.1)):
			twist_obj=Twist()
			twist_obj.linear.x=0
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=0.5
			print twist_obj.angular.z
			self.vel_pub.publish(twist_obj)
			

			if self.server.is_preempt_requested():
				result = RotateToGoalResult()
				result.result = False
				self.server.set_preempted(result, "preempted")
				return


		if((abs(self.current_yaw-self.dest_yaw)<0.1)):
			flag=0
			twist_obj=Twist()
			twist_obj.linear.x=0
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=0.0
			self.vel_pub.publish(twist_obj)
			
			result = RotateToGoalResult()
			result.result = True
			self.server.set_succeeded(result, "Goal reached successfully")


			#feedback
			feedback = RotateToGoalFeedback()
			feedback.angle_left = abs(self.current_yaw-self.dest_yaw)
			self.server.publish_feedback(feedback)

		#return from here so that server.set_succeeded executes

	def init_twist(self):
		print("In init twist")
		self.pose.linear.x = 0
		self.pose.linear.y = 0
		self.pose.linear.z = 0
		self.pose.angular.x = 0
		self.pose.angular.y = 0
		self.pose.angular.z = 0




		


if __name__=="__main__":
	rospy.init_node("husky_rotator")
	server = RotateToGoalServer()
	print("Ready.")
