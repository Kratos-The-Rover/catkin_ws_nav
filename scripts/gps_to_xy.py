#!/usr/bin/env python
import sys
import os
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Pose
from navigation.msg import Point_xy
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './rrt_for_scan')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './rrt_star')))
from utils_scan import local_to_global_pt

import utm,math
current_gps=NavSatFix()
pub=0
pose=Pose()
def odom_cb(data):
	global pose
	pose=data.pose.pose

def convert_utm(data):
	global pub
	a=data.latitude
	b=data.longitude
	c=current_gps.latitude
	d=current_gps.longitude
	q,w,e,r= utm.from_latlon(a,b)
	t,y,u,i= utm.from_latlon(c,d)
	local_x=t-q
	local_y=y-w
	#print(q,w ,"to" ,t,y)
	#print(t-q,y-w)
	#print(math.sqrt((t-q)**2+(y-w)**2))

	pub.publish(Point_xy(local_to_global_pt([local_x,local_y],pose)))


def callback(data):
	global current_gps
	current_gps=data
	
def listener():
	global pub
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('gps_to_xy', anonymous=True)

	rospy.Subscriber("/gps_bot", NavSatFix, callback)
	rospy.Subscriber("/gps_goal",NavSatFix,convert_utm)
	rospy.Subscriber("/odom",Odometry,odom_cb)
	pub = rospy.Publisher('/goal2', Point_xy, queue_size=10)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()
