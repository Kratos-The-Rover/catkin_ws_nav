#! /usr/bin/env python
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './rrt_for_scan')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './rrt_star')))
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan,NavSatFix
from shapely.geometry import Polygon, Point, LineString
from utils_scan import check_intersection_scan,make_obstacles_scan
from nav_msgs.msg import Odometry
from navigation.srv import *
from geometry_msgs.msg import Pose
import roslib
roslib.load_manifest('navigation')
import actionlib

from navigation.msg import moveToGoalAction, moveToGoalGoal, RotateToGoalGoal, RotateToGoalAction, Point_xy, PointArray
#import service module and message file



class RootClient(object):
	"""
	Root class for Controller Client
	
	  Attributes:
		Services Available for this Client:
			~ rrt_star_planner_service : navigation/start navigation/goal sensor_msgs/LaserScan | navigation/PointArray bool
			~ dynamic_planner_service : navigation/start navigation/goal sensor_msgs/LaserScan |  bool
			~ commander : navigation/current_pt navigation/next_pt | bool | float64(Distance left to travel)  
	
	"""
	
	def __init__(self, *args , **kwargs):
		print("Initializing Subscriber/Publishers/Actions/Services/Variables")
		"""
		Initialize all ROS services, actions and Pub/Sub
		Service to be called --> rrt_star_planner_service , dynamic_planner_service  
		Subscribed Topics --> /scan /odom  
		Published Topics --> 
		Parameters Set -->
		"""
		#rrt_star Planner Service 

		#class variables initialized:

		self.start = Point_xy([0,0])
		self.goal = Point_xy([5,4])
		self.final_dest = [0,0]
		self.scan_list = []

		self.current_pos = Pose()
		self.currentgps = NavSatFix()

		nan=100
		# self.scan_list =[1.309682011604309, 1.3101788759231567, 1.310431718826294, 1.3106871843338013, 1.3109458684921265, 1.311471939086914, 1.311739444732666, 1.3120099306106567, 1.3122836351394653, 1.3125602006912231, 1.3128397464752197, 1.3134082555770874, 1.313697099685669, 1.3139890432357788, 1.3142839670181274, 1.3145822286605835, 1.3151880502700806, 1.3154956102371216, 1.3158066272735596, 1.3161206245422363, 1.316437840461731, 1.316758394241333, 1.3174091577529907, 1.317739486694336, 1.3180732727050781, 1.318410038948059, 1.3187503814697266, 1.3190940618515015, 1.3194409608840942, 1.3201451301574707, 1.3205022811889648, 1.3208627700805664, 1.3212268352508545, 1.3215943574905396, 1.3219653367996216, 1.3223397731781006, 1.3230992555618286, 1.3234843015670776, 1.3238729238510132, 1.3242650032043457, 1.3246607780456543, 1.3250601291656494, 1.325463056564331, 1.3258696794509888, 1.3266940116882324, 1.3271117210388184, 1.32753324508667, 1.327958583831787, 1.3283874988555908, 1.3288201093673706, 1.3292566537857056, 1.3296970129013062, 1.3301411867141724, 1.3310409784317017, 1.3314965963363647, 1.331956148147583, 1.332419753074646, 1.3328871726989746, 1.3333585262298584, 1.333833932876587, 1.3343132734298706, 1.3347965478897095, 1.335283875465393, 1.3362706899642944, 1.3367700576782227, 1.3372735977172852, 1.3377811908721924, 1.3382928371429443, 1.3388087749481201, 1.3393288850784302, 1.339853048324585, 1.340381383895874, 1.3409141302108765, 1.3414509296417236, 1.3419921398162842, 1.342537522315979, 1.3436414003372192, 1.3441998958587646, 1.3447626829147339, 1.3453298807144165, 1.345901370048523, 1.3464775085449219, 1.347057819366455, 1.3476428985595703, 1.3482322692871094, 1.3488261699676514, 1.3494247198104858, 1.3500275611877441, 1.3506351709365845, 1.3512473106384277, 1.3518640995025635, 1.3524854183197021, 1.3531115055084229, 1.353742241859436, 1.3543776273727417, 1.35566246509552, 1.3563120365142822, 1.356966495513916, 1.3576256036758423, 1.3582897186279297, 1.3589587211608887, 1.3596327304840088, 1.360311508178711, 1.3609951734542847, 1.361683964729309, 1.362377405166626, 1.363076090812683, 1.3637796640396118, 1.3644883632659912, 1.3652020692825317, 1.365920901298523, 1.3666447401046753, 1.3673737049102783, 1.3681076765060425, 1.3688467741012573, 1.3695908784866333, 1.370340347290039, 1.3710949420928955, 1.3718549013137817, 1.3726199865341187, 1.3733903169631958, 1.3741658926010132, 1.3749470710754395, 1.375733494758606, 1.3765252828598022, 1.3773225545883179, 1.3781251907348633, 1.378933310508728, 1.379746913909912, 1.380566120147705, 1.3813908100128174, 1.382220983505249, 1.3830569982528687, 1.3838989734649658, 1.3847463130950928, 1.3855993747711182, 1.386458158493042, 1.3873226642608643, 1.3881930112838745, 1.3890687227249146, 1.3899502754211426, 1.3908374309539795, 1.3917304277420044, 1.3926293849945068, 1.3935341835021973, 1.3944448232650757, 1.3953614234924316, 1.3962838649749756, 1.3972123861312866, 1.3981468677520752, 1.3990874290466309, 1.4000341892242432, 1.4009875059127808, 1.401947021484375, 1.4029126167297363, 1.4038845300674438, 1.404862403869629, 1.4058465957641602, 1.406836986541748, 1.4078336954116821, 1.4088366031646729, 1.4098458290100098, 1.4108614921569824, 1.4118835926055908, 1.4129116535186768, 1.4139459133148193, 1.4149869680404663, nan, 1.41603422164917, 1.4170881509780884, 1.4181488752365112, 1.4192163944244385, 1.4202903509140015, 1.4213711023330688, 1.4224584102630615, 1.4235525131225586, 1.42465341091156, 1.4257612228393555, 1.4268758296966553, 1.4279974699020386, 1.4291259050369263, 1.4302610158920288, 1.4314026832580566, 1.4325510263442993, 1.433706283569336, 1.434868574142456, 1.4360378980636597, 1.4372141361236572, 1.438397765159607, 1.439588189125061, nan, 1.4407858848571777, 1.441990613937378, 1.4432027339935303, 1.4444221258163452, 1.4456491470336914, 1.4468833208084106, 1.4481250047683716, 1.4493738412857056, 1.4506300687789917, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan]
		
   
		self.currentOdom = Odometry()


		self.scan_sub = rospy.Subscriber('/scan', LaserScan , self.scan_cb,queue_size=10)
		self.odom_sub = rospy.Subscriber('/odom', Odometry , self.odom_cb)
		#self.gps_Sub = rospy.Subscriber('global_position/local' , NavSatFix , self.gps_cb)
		self.rrt_star_path = rospy.ServiceProxy('rrt_planner', Planner)
		self.move_client = actionlib.SimpleActionClient('commander', moveToGoalAction)
		self.move_client.wait_for_server()
		self.rotate_client = actionlib.SimpleActionClient('rotator',RotateToGoalAction)
		self.rotate_client.wait_for_server()
		self.dynamic_manager = rospy.ServiceProxy('dynamic_planner_service', Dynamic)
		rate=rospy.Rate(1)
		rate.sleep()

		print("Done initializing Subscriber/Publishers/Actions/Services/Variables")

	def scan_cb(self , scan_data):
		"""
		Args: scan_data --> sensor_msgs/LaserScan
		Attributes: sets self.scan_list
		Return: None
		"""
		#print("In Scan_CB")
		self.scan_list = scan_data.ranges
		self.scan_list = list(self.scan_list)
		for i in range(len(self.scan_list)):
			if(math.isnan(self.scan_list[i])):
				self.scan_list[i] = 100
		self.scan_list = tuple(self.scan_list)
		# self.scan_list = list(scan_list)
		# for i in range(len(self.scan_list)):
		#     if(math.isnan(self.scan_list[i])):
		#     #converting nan values to some number
		#         self.scan_list[i] = 1000            
		# self.scan_list = tuple(self.scan_list)
		#print("Scan_CB DONE")

	def odom_cb(self, odom_data):
		#print("In Odom_CB")
		"""
		Args: odom_data --> nav_msgs/Odometry
		Attributes: sets self.currentOdom
		Return: None 
		"""
		self.currentOdom = odom_data
		#print("Odom_CB DONE")

	def gps_cb(self , gps_data):
		"""
		Args: gps_data --> sensor_msgs/NavSatFix
		Attributes: sets self.currentOdom
		Return: None 
		"""
		self.currentgps = gps_data
	
	def commander_fb(self, fb_data):
		# print("In Commander_FB")
		"""
		Args: feedback from commander
		Attributes: prints feedback from commander
		Return: None 
		"""
		#rospy.loginfo(fb_data.distance_left)
		# print("Commander_FB DONE")
		pass
	
	def rotator_fb(self, fb_data):
		# print("In Rotator_FB")
		"""
		Args: feedback from commander
		Attributes: prints feedback from commander
		Return: None 
		"""
		#rospy.loginfo(fb_data.angle_left)
		# print("ROtator_FB DONE")X
		pass
	

	def global_to_local_goal(self,pt_in_global):
		x2=pt_in_global.point[0]
		y2=pt_in_global.point[1]
		x1=self.currentOdom.pose.pose.position.x
		y1=self.currentOdom.pose.pose.position.y
		y_diff=y2-y1
		x_diff=x2-x1
		#dest_yaw = math.atan2(y_diff,x_diff)
		orientation_q = self.currentOdom.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, curr_yaw) = euler_from_quaternion (orientation_list)
		#theta=dest_yaw-curr_yaw
		#r=math.sqrt(x_diff**2+y_diff**2)
		x_ret=x_diff*math.cos(curr_yaw)+y_diff*math.sin(curr_yaw)
		y_ret=-x_diff*math.sin(curr_yaw)+y_diff*math.cos(curr_yaw)
		return Point_xy([x_ret,y_ret])
	

	def local_to_global_path(self,path_in_local):
		path_in_global=PointArray()
		x1=self.currentOdom.pose.pose.position.x
		y1=self.currentOdom.pose.pose.position.y
		orientation_q = self.currentOdom.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, theta1) = euler_from_quaternion (orientation_list)
		
		for i in range(len(path_in_local.points)):
			x=path_in_local.points[i].point[0]
			y=path_in_local.points[i].point[1]
			theta2=math.atan2(y,x)
			theta=theta1+theta2
			r=math.sqrt(x**2+y**2)
			x2=r*math.cos(theta)
			y2=r*math.sin(theta)
			path_in_global.points.append(Point_xy([x1+x2,y1+y2]))
		return path_in_global


	def main(self):
		print("In MAIN")
		"""
		Args:
		Attributes: sets self.currentOdom
		Return:
		"""
		# rospy.set_param('start_location',"["+str(self.currentgps.latitude)+","+str(self.currentgps.longitude)+"]")

		# while not rospy.is_shutdown:
		flag=1
		while(flag):
			final_path = []
			# self.goal = rospy.get_param('')
			try:
				print("Calculating RRT-PATH")
				local_goal=self.global_to_local_goal(self.goal)
				print("LOCAL GOAL",local_goal)
				response = self.rrt_star_path(Point_xy([0,0]),local_goal,self.scan_list)
				if response.ack:
					local_path = response.path
					print("local path",local_path)
					final_path = self.local_to_global_path(local_path)
					print("path calculated",final_path)
				else:
					print("Path not found! Retrying!")
					continue
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

			while len(final_path.points)!=1:
				#rotate towards goal
				print("Rotating the bot")
				goal2 = RotateToGoalGoal()
				goal2.goal = final_path.points[-2]
				self.rotate_client.send_goal(goal2, feedback_cb = self.rotator_fb)
				self.rotate_client.wait_for_result(rospy.Duration.from_sec(100.0))
				print("Done rotating the bot")
				
				# rospy.loginfo("Asking the bot to move.")
				print("Calling dynamic manager service")
				x1=final_path.points[-1].point[0]
				y1=final_path.points[-1].point[1]
				x2=final_path.points[-2].point[0]
				y2=final_path.points[-2].point[1]
				dist=math.sqrt((x2-x1)**2+(y2-y1)**2)
				response_1 = self.dynamic_manager(Point_xy([0,0]),Point_xy([dist,0]), self.scan_list)
				#response_1 = self.dynamic_manager(final_path.points[-1],final_path.points[-2],self.scan_list)
				if response_1.ack:
					print("No intersection!!!!")
					#call commander to move the bot 
					print("Moving the bot")
					goal1 = moveToGoalGoal()
					goal1.goal = final_path.points[-2]
					self.move_client.send_goal(goal1, feedback_cb = self.commander_fb)
					# rospy.loginfo("Asking the bot to move.")
					self.move_client.wait_for_result(rospy.Duration.from_sec(100.0))
					print("Bot has moved")

					#remove first node when successful
					print("Final path updated")
					final_path.points = final_path.points[:-1]
				else:
					# try:
					# 	print("Recalculating RRT-Path")
					# 	response = self.rrt_star_path(final_path.points[0],self.goal,self.scan_list)
					# 	if response.ack:
					# 		print("Path Recalculated")
					# 		final_path = response.path
					# 	else:
					# 		continue
					# except rospy.ServiceException, e:
					# 	print "Service call failed: %s"%e
					break
				print("Length of final_path",len(final_path.points))
			if(len(final_path.points)==1):
				flag=0

		rospy.signal_shutdown("************************GOAL REACHED************************")
	

if __name__ == "__main__":
	rospy.init_node("controller_client", anonymous=True)
	print("Node initialized")
	try:
		print("Waiting for services.")
		rospy.wait_for_service('rrt_planner',5)
		rospy.wait_for_service('dynamic_planner_service',5)
		# rospy.wait_for_service('commander',5)
		# rospy.wait_for_service('rotator',5)
		print("Done waiting for services")
	except rospy.ServiceException, e:
		rospy.logerr("Services Could not be initialized")
	print("Calling RootClient")    
	o = RootClient()
	print("Calling Main")
	o.main()
	print("RosSpin")
	rospy.spin()
	rospy.logwarn("Killing!")