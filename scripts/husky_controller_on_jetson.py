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
from navigation.msg import moveToGoalAction, moveToGoalGoal, moveToGoalResult, moveToGoalFeedback, Point_xy
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
flag=1
class moveToGoalServer():

	def __init__(self):
		print("in constructor-moveToGoalServer ")
		self.server = actionlib.SimpleActionServer('commander', moveToGoalAction, self.execute, False)
		self.server.start()
		self.flag=1
		print("Ready in moveToGoalServer")
		rospy.spin()

	def execute(self,goal):
		while(flag):
			print("executing")
			self.fwd_vel=0.3
			self.k1 = 2
			self.p = goal.goal.point
			self.dest_yaw = self.p[1]/(math.sqrt(math.pow(self.p[0],2)+math.pow(self.p[1],2)))
			my_controller = husky_controller(self.server,self.fwd_vel,self.k1,self.p,self.dest_yaw)

			print("execution finished")

class husky_controller():
	def __init__(self,server,fwd_vel,k1,p,dest_yaw):
		print("in constructor-husky")
		self.pose = Twist()
		self.odom = Odometry()
		self.server = server
		self.dest_x = p[0]
		self.dest_y = p[1]
		self.dest_yaw = dest_yaw
		self.fwd_vel = fwd_vel
		self.k1 = k1
		
		self.init_twist()


		#to be changed
		# self.vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)
		# self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_cb)
		# rate=rospy.Rate(100)

		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size=10)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
		rate=rospy.Rate(100)
		rate.sleep()


	def odom_cb(self,data):
		print("message received")
		self.pose.linear.x = data.pose.pose.position.x
		self.pose.linear.y = data.pose.pose.position.y
		self.pose.linear.z = data.pose.pose.position.z
		# print data.pose.pose.position.x
		# print data.pose.pose.position.y
		print "x   ",abs(self.pose.linear.x-self.dest_x)
		print "y   ",abs(self.pose.linear.y-self.dest_y)
		quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
		euler=tf.transformations.euler_from_quaternion(quaternion)

		self.pose.angular.x = euler[0]
		self.pose.angular.y = euler[1]
		self.pose.angular.z = euler[2]

		current_yaw=self.pose.angular.z
		print()
		x_dist = abs(self.pose.linear.x-self.dest_x)
		y_dist = abs(self.pose.linear.y-self.dest_y)
		print("self.pose.linear.x=",self.pose.linear.x)
		print("self.pose.linear.y=",self.pose.linear.y)
		print("dest_x=",self.dest_x)
		print("dest_y=",self.dest_y)

		# if(x_dist<0.5) and y_dist<0.5)):
		# 	print "condition satisfied"
		# 	pwm0.start(0)
		# 	pwm1.start(0)
		# 	pwm2.start(0)
		# 	pwm3.start(0)
		# else:
		# 	v=100
		# 	def mymap(a,b,c.d,e):
		# 		return d+((a-b)*(e-d)/(c-b))
			
		# 	vel_wheels[0] = v + (k1*(dest_yaw-current_yaw))
  #   		vel_wheels[1] = v + (k1*(dest_yaw-current_yaw))
  #   		vel_wheels[2] = v - (k1*(dest_yaw-current_yaw))
  #   		vel_wheels[3] = v - (k1*(dest_yaw-current_yaw))

  #   		pwmvel[0]=mymap(vel_wheels[0],v-k1,v+k1,0,100)
  #   		pwmvel[1]=mymap(vel_wheels[1],v-k1,v+k1,0,100)
  #   		pwmvel[2]=mymap(vel_wheels[2],v-k1,v+k1,0,100)
  #   		pwmvel[3]=mymap(vel_wheels[3],v-k1,v+k1,0,100)
  #   		pwm0.start(pwmvel[0])
		# 	pwm1.start(pwmvel[1])
		# 	pwm2.start(pwmvel[2])
		# 	pwm3.start(pwmvel[3])

		print("x_dist=",x_dist)
		print("y_dist=",y_dist)

		if((x_dist<0.5) and (y_dist<0.5)):
			print "condition satisfied"
			twist_obj=Twist()
			twist_obj.linear.x=0
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=0
			self.vel_pub.publish(twist_obj)
			result = moveToGoalResult()
			result.result = True
			self.server.set_succeeded(result, "Goal reached successfully")
			flag=0


		else:
			print("In else")
			twist_obj=Twist()
			twist_obj.linear.x=self.fwd_vel
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=self.k1*(self.dest_yaw-current_yaw)
			self.vel_pub.publish(twist_obj)

			if self.server.is_preempt_requested():
				result = moveToGoalResult()
				result.result = False
				self.server.set_preempted(result, "preempted")
				return

			#feedback
			feedback = moveToGoalFeedback()
			feedback.distance_left = math.sqrt((x_dist)**2 + (y_dist)**2)
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
	rospy.init_node("husky_controller")
	server = moveToGoalServer()
	print("Ready.")
