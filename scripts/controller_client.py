#! /usr/bin/env python

import numpy as np
import sys
import os
import rospy
from sensor_msgs.msg import LaserScan
from shapely.geometry import Polygon, Point, LineString
from utils_scan import check_intersection_scan,make_obstacles
from nav_msgs.msg import Odometry
import roslib
roslib.load_manifest('navigation')
import actionlib

from navigation.msg import moveToGoalAction, moveToGoalGoal
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
		"""
		Initialize all ROS services, actions and Pub/Sub
        Service to be called --> rrt_star_planner_service , dynamic_planner_service  
		Subscribed Topics --> /scan /odom  
        Published Topics --> 
        Parameters Set -->
        """
		#rrt_star Planner Service 

        self.scan_sub = rospy.Subscriber('/scan', LaserScan , self.scan_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry , self.odom_cb)
        self.gps_Sub = rospy.Subscriber('global_position/local' , NavSatFix , self.gps_cb)
        self.rrt_star_path = rospy.ServiceProxy('rrt_planner', Planner)
        self.client = actionlib.SimpleActionClient('commander', moveToGoalAction)
        self.dynamic_manager = rospy.ServiceProxy('dynamic_planner_service', Dynamic)

        #class variables initialized:

        self.start = [0,0]
        self.goal = [0,0]
        self.final_dest = [0,0]
        self.flag = 0

        self.current_pos = Pose()
        self.currentgps = NavSatFix()

        self.scan_list = []
        self.currentOdom = Odometry()

    def scan_cb(self , scan_data):
        """
        Args: scan_data --> sensor_msgs/LaserScan
        Attributes: sets self.scan_list
        Return: None
        """
        self.scan_list = scan_data.ranges
        scan_temp = np.array(scan_data.ranges)
        self.scan_list = scan_temp[scan_temp == 'inf']=1000
            
        # self.scan_list = list(scan_list)
        # for i in range(len(self.scan_list)):
        #     if(math.isnan(self.scan_list[i])):
        #     #converting nan values to some number
        #         self.scan_list[i] = 1000            
        # self.scan_list = tuple(self.scan_list)

    def odom_cb(self, odom_data):
        """
        Args: odom_data --> nav_msgs/Odometry
        Attributes: sets self.currentOdom
        Return: None 
        """
        self.currentOdom = odom_data

    def gps_cb(self , gps_data):
        """
        Args: gps_data --> sensor_msgs/NavSatFix
        Attributes: sets self.currentOdom
        Return: None 
        """
        self.currentgps = gps_data
    
    def commander_fb(self, fb_data):
        """
        Args: feedback from commander
        Attributes: prints feedback from commander
        Return: None 
        """
        rospy.loginfo(fb_data.distance_left)
    
    def main():
        """
        Args:
        Attributes: sets self.currentOdom
        Return:
        """
        # rospy.set_param('start_location',"["+str(self.currentgps.latitude)+","+str(self.currentgps.longitude)+"]")

        while not rospy.is_shutdown:
            
            # self.goal = rospy.get_param('')
            try:
                response = self.rrt_star_path(self.start,self.goal,self.scan_list)
                if response.ack:
                    final_path = response.path
                    self.flag = 1
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            while len(final_path)!=1:
                response_1 = self.dynamic_manager(self.final_path[0],self.final_path[1],self.scan_list)
                if response_1.ack:
                    #call commander to move the bot 
                    goal1 = moveToGoalGoal()
                    goal1.goal = Point_xy([final_path[1]])
                    self.client.send_goal(goal1)
                    # rospy.loginfo("Asking the bot to move.")
                    self.client.wait_for_result(rospy.Duration.from_sec(100.0), feedback_cb = self.commander_fb)
                    # print("Bot has moved")

                    #remove first node when successful
                    final_path = final_path[1:]
                else:
                    try:
                        response = self.rrt_star_path(final_path[0],self.goal,self.scan_list)
                        if response.ack:
                            final_path = response.path
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e


if __name__ == "__main__":
    rospy.init_node("controller_client", anonymous=True)
    try:
        rospy.wait_for_service('rrt_star_planner_service',5)
        rospy.wait_for_service('dynamic_planner_service',5)
        rospy.wait_for_service('commander',5)
    except rospy.ServiceException, e:
        rospy.logerr("Services Could not be initialized")    
    o = RootClient()
    o.main()
    rospy.spin()
	rospy.logwarn("Killing!")