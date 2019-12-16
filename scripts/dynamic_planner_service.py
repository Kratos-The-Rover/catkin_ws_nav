#!/usr/bin/env python
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './rrt_for_scan')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './rrt_star')))
from navigation.srv import Dynamic,DynamicResponse, DynamicRequest
from shapely.geometry import Polygon, Point , LineString
import rospy
from utils_scan import make_obstacles_scan,check_intersection_scan

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../rrt_for_scan/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../rrt_star/")


def chk_intersection(req):
	line_obstacles,pts = make_obstacles_scan(req.scan_list)
	for obstacle in line_obstacles:
		print(check_intersection_scan([req.start_pos.point,req.goal_pos.point],line_obstacles))
		if check_intersection_scan([req.start_pos.point,req.goal_pos.point],line_obstacles):
			return DynamicResponse(False)
	return DynamicResponse(True)

def obstacle_check():
	rospy.init_node('obstacle_check', anonymous=True)
	s = rospy.Service('dynamic_planner_service', Dynamic, chk_intersection)
	print "Ready."
	rospy.spin()


if __name__ == '__main__':
	obstacle_check()
