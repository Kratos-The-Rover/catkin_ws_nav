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
        self.client = actionlib.SimpleActionClient('commander', moveToGoalAction)
        #class variables initialized:

        self.start = [0,0]
        self.goal = [0,0]
        
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
    
    def main():
        """
        Args:
        Attributes: sets self.currentOdom
        Return:
        """
        # rospy.set_param('start_location',"["+str(self.currentgps.latitude)+","+str(self.currentgps.longitude)+"]")

        while not rospy.is_shutdown:
                
            self.scan_list = list(scan_list)
            for i in range(len(self.scan_list)):
                if(math.isnan(self.scan_list[i])):
                #converting nan values to some number
                    self.scan_list[i] = 1000            
            self.scan_list = tuple(self.scan_list)

            # self.goal = rospy.get_param('')
            if(self.flag==0)
                try:
                    rrt_star_path = rospy.ServiceProxy('rrt_planner', Planner)
                    response = rrt_star_path(self.start,self.goal,self.scan_list)
                    if response.ack:
                        final_path = response.path
                        self.flag = 1
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

            try:
                dynamic_manager = rospy.ServiceProxy('dynamic_planner_service', Dynamic)
                response_1 = dynamic_manager(self.final_path[0],self.final_path[1],self.scan_list)
                if response_1.ack:
                    #call commander to move the bot 
                    goal1 = moveToGoalGoal()
                    goal1.goal = Point_xy([final_path[1]])
                    client.send_goal(goal1)
                    print("Asking the bot to move.")
                    client.wait_for_result(rospy.Duration.from_sec(100.0))
                    print("Bot has moved")

                    #remove first node when successful
                    final_path = final_path[1:]
                else:
                    #exit the function and call RRT service again for a new path
                    self.flag = 0
                    break
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    



if __name__ == "__main__":
    rospy.init_node("controller_client", anonymous=True)
	rospy.wait_for_service('rrt_star_planner_service')
    rospy.wait_for_service('dynamic_planner_service')
    rospy.wait_for_service('commander')
    
    o = RootClient()
    o.main()
	rospy.logwarn("Killing!")