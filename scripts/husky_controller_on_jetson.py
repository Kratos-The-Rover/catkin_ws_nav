#import Jetson.GPIO as GPIO
#import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import math
import tf
import roslib
roslib.load_manifest('navigation')
import actionlib
from navigation.msg import moveToGoalAction

dirs[4]={}
pwm[4]={}

GPIO.setmode(GPIO.BOARD)

GPIO.setup(dirs[0], GPIO.OUT)
GPIO.setup(dirs[1], GPIO.OUT)
GPIO.setup(dirs[2], GPIO.OUT)
GPIO.setup(dirs[3], GPIO.OUT)

GPIO.setup(pwm[0], GPIO.OUT)
GPIO.setup(pwm[1], GPIO.OUT)
GPIO.setup(pwm[2], GPIO.OUT)
GPIO.setup(pwm[3], GPIO.OUT)

pwm0 = GPIO.PWM(pwm[0], 2000)
pwm1 = GPIO.PWM(pwm[1], 2000)
pwm2 = GPIO.PWM(pwm[2], 2000)
pwm3 = GPIO.PWM(pwm[3], 2000)


fwd_vel=0.1


class moveToGoalServer():

	def __init__(self):
		self.server = actionlib.SimpleActionServer('move_to_goal', moveToGoalAction, self.execute, False)
		self.server.start()

	def execute(self,goal):
		fwd_vel=0.5
		k1 = 8
		p = goal.point
		dest_x = p[0]
		dest_y = p[1]
		dest_yaw = dest_y/(math.sqrt(math.pow(dest_x,2)+math.pow(dest_y,2)))
		my_controller = husky_controller()
		# self.server.set_succeeded()

class husky_controller():
	def __init__(self):
		self.pose = Twist()
		self.odom = Odometry()

		self.init_twist()

		#to be changed
		self.vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)
		self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_cb)
		rate=rospy.Rate(100)


	def odom_cb(self,data):
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

		current_yaw=self.pose.angular.z

		if((abs(self.pose.linear.x-dest_x)<0.5) and (abs(self.pose.linear.y-dest_y)<0.5)):
			print "condition satisfied"
			pwm0.start(0)
			pwm1.start(0)
			pwm2.start(0)
			pwm3.start(0)
		else:
			v=100
			def mymap(a,b,c.d,e):
				return d+((a-b)*(e-d)/(c-b))
			
			vel_wheels[0] = v + (k1*(dest_yaw-current_yaw))
    		vel_wheels[1] = v + (k1*(dest_yaw-current_yaw))
    		vel_wheels[2] = v - (k1*(dest_yaw-current_yaw))
    		vel_wheels[3] = v - (k1*(dest_yaw-current_yaw))

    		pwmvel[0]=mymap(vel_wheels[0],v-k1,v+k1,0,100)
    		pwmvel[1]=mymap(vel_wheels[1],v-k1,v+k1,0,100)
    		pwmvel[2]=mymap(vel_wheels[2],v-k1,v+k1,0,100)
    		pwmvel[3]=mymap(vel_wheels[3],v-k1,v+k1,0,100)
    		pwm0.start(pwmvel[0])
			pwm1.start(pwmvel[1])
			pwm2.start(pwmvel[2])
			pwm3.start(pwmvel[3])


		if((abs(self.pose.linear.x-dest_x)<0.5) and (abs(self.pose.linear.y-dest_y)<0.5)):
			print "condition satisfied"
			twist_obj=Twist()
			twist_obj.linear.x=0
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=0
			self.vel_pub.publish(twist_obj)


		else:
			twist_obj=Twist()
			twist_obj.linear.x=fwd_vel
			twist_obj.linear.y=0
			twist_obj.linear.z=0
			twist_obj.angular.x=0
			twist_obj.angular.y=0
			twist_obj.angular.z=k1*(dest_yaw-current_yaw)
			self.vel_pub.publish(twist_obj)

		#return from here so that server.set_succeeded executes

	def init_twist(self):
		self.pose.linear.x = 0
		self.pose.linear.y = 0
		self.pose.linear.z = 0
		self.pose.angular.x = 0
		self.pose.angular.y = 0
		self.pose.angular.z = 0




		


if __name__=="__main__":
	rospy.init_node("husky_controller")
	server = moveToGoalServer()
	rospy.spin()