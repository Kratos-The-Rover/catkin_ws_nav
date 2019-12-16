#! /usr/bin/env python

import numpy as np
import sys
import os
import rospy
from sensor_msgs.msg import LaserScan
from shapely.geometry import Polygon, Point, LineString
from utils_scan import check_intersection_scan,make_obstacles
#import service module and message file

path = []


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print("Start " + __file__)
    #inf  = 100
    # ====Search Path with RRT====
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

    



def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('controller_client', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)
	
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

   
	


if __name__ == '__main__':
    main()




def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



