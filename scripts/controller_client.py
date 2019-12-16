#! /usr/bin/env python

import numpy as np
import sys
import os
import rospy
from sensor_msgs.msg import LaserScan
from shapely.geometry import Polygon, Point, LineString
from utils_scan import check_intersection_scan,make_obstacles
from nav_msgs.msg import Odometry
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
        self.gps_Sub = rospy.Subscriber('/')
        #class variables initialized:

        self.start = [0,0]
        self.goal = [0,0]
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


    def main():
        """
        Args:
        Attributes: sets self.currentOdom
        Return:
        """
        scan_list = data.ranges
        scan_list = list(scan_list)
        for i in range(len(scan_list)):
            if(math.isnan(scan_list[i])):
            #converting nan values to some number
                scan_list[i] = 1000            
        scan_list = tuple(scan_list)
        line_obstacles, pts = make_obstacles_scan(scan_list)
        
        if len(path)>=2:
            curr_x = path[0][0]
            curr_y = path[0][1]
            next_x = path[1][0]
            next_y = path[1][1]
            
        point_list = [(curr_x,curr_y),(next_x,next_y)]
        if not check_intersection_scan(point_list , line_obstacles):
            #move forward
            
            #rotate towards next point

            #remove current point from path
            path = path[1:]
            return


        rospy.wait_for_service('rrt_star_planner_service')
        try:
            final_path = rospy.ServiceProxy('rrt_star_planner_service', Planner)
            #start position is current position
            resp = final_path(scan_list,start_position,goal_position)
        if resp.ack:
                
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    



if __name__ == "__main__":
    rospy.init_node("controller_client", anonymous=True)
	rospy.wait_for_service('rrt_star_planner_service')
    rospy.wait_for_service('dynamic_planner_service')
    # rospy.wait_for_service() # SET THIS FOR COMMANDER
    o = RootClient()
    o.main()
	rospy.logwarn("Killing!")